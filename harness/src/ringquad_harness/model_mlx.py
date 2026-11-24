from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import sys
import time
from typing import Literal, Sequence

from pydantic import BaseModel, ConfigDict
from pydantic_ai import Agent, PromptedOutput as PydanticPromptedOutput
from pydantic_ai.messages import ModelResponse, TextPart
from pydantic_ai.models.function import AgentInfo, FunctionModel

from .schemas import Decision, EvidenceFrame, Observation


class PromptedOutput(BaseModel):
    model_config = ConfigDict(extra="forbid")
    explanation: str
    outcome: Literal["inspect", "monitor", "insufficient_evidence"]


_WORKER = r'''
import json, sys
from pathlib import Path
from PIL import Image
from mlx_vlm import load, generate
from mlx_vlm.prompt_utils import apply_chat_template
from mlx_vlm.structured import build_json_schema_logits_processor
from pydantic import BaseModel, ConfigDict
from typing import Literal
class Output(BaseModel):
    model_config = ConfigDict(extra="forbid")
    explanation: str
    outcome: Literal["inspect", "monitor", "insufficient_evidence"]
request=json.load(sys.stdin)
model, processor=load(request['model_path'])
schema=Output.model_json_schema()
images=[Image.open(request['context']).convert('RGB'), Image.open(request['crop']).convert('RGB')]
prompt=apply_chat_template(processor, model.config, request['prompt'], num_images=len(images))
processor_fn=build_json_schema_logits_processor(processor.tokenizer, schema)
result=generate(model, processor, prompt, image=[request['context'], request['crop']], max_tokens=220, temperature=0.0, logits_processors=[processor_fn], verbose=False)
print(json.dumps({'text':result.text, 'peak_memory_gb':getattr(result, 'peak_memory', None)}))
'''


async def _communicate_cancel_safe(proc, payload: bytes):
    """Send input, drain both output streams, and reap even when our caller cancels."""
    communication = asyncio.create_task(proc.communicate(payload))
    try:
        return await asyncio.shield(communication)
    except asyncio.CancelledError:
        if proc.stdin and not proc.stdin.is_closing():
            proc.stdin.close()
        try:
            proc.kill()
        except ProcessLookupError:
            pass
        try:
            await asyncio.shield(communication)
        except asyncio.CancelledError:
            try:
                await proc.wait()
            except ProcessLookupError:
                pass
        except Exception:
            try:
                await proc.wait()
            except ProcessLookupError:
                pass
        raise


class MLXReviewBackend:
    """One child process per inference so timeout cancellation can reap MLX."""

    def __init__(self, model_path: Path, *, timeout: float = 60.0, python: str | None = None):
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_dir():
            raise FileNotFoundError(f"local MLX model directory does not exist: {self.model_path}")
        self.timeout = timeout
        self.python = python or sys.executable
        self.metadata = {"provider": "mlx-vlm", "model": "mlx-community/Qwen3-VL-2B-Instruct-4bit",
                         "local_path": str(self.model_path), "revision": self.model_path.name}

    @staticmethod
    def _paths(evidence: Sequence[EvidenceFrame]) -> tuple[EvidenceFrame, Path, Path]:
        if not evidence:
            raise ValueError("observation has no saved evidence")
        selected = evidence[-1]
        return selected, Path(selected.context_path).resolve(), Path(selected.crop_path).resolve()

    async def review(self, observation: Observation, evidence: Sequence[EvidenceFrame]) -> Decision:
        self.last_metrics = {}
        selected, context, crop = self._paths(evidence)
        if not context.is_file() or not crop.is_file():
            raise ValueError("selected evidence image is missing")
        prompt = (
            "You are reviewing a candidate canopy discoloration for agricultural scouting. "
            "Image 1 is the entire frame, image 2 is its crop around the candidate. Describe visible "
            "evidence first. Choose inspect if a visible localized color change in the crop warrants "
            "taking a closer image, even if the cause is unknown. Choose monitor if readable with no "
            "localized discoloration. Choose insufficient_evidence if blur, shadows, or occlusion "
            "prevent deciding. Do not diagnose disease. Keep explanation brief. Return JSON in order: "
            f"explanation, outcome. Observation {observation.observation_id}."
        )
        payload = {"model_path": str(self.model_path), "context": str(context), "crop": str(crop), "prompt": prompt}
        async def async_adapter(messages, _: AgentInfo) -> ModelResponse:
            # The model process is owned by this coroutine; wait_for cancellation kills and
            # reaps it, including when the outer review timeout expires.
            proc = await asyncio.create_subprocess_exec(
                self.python, "-c", _WORKER, stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                env={**os.environ, "PYDANTIC_AI_NO_BANNER": "1", "HF_HUB_OFFLINE": "1",
                     "TRANSFORMERS_OFFLINE": "1"},
            )
            try:
                started = time.monotonic()
                stdout, stderr = await _communicate_cancel_safe(proc, json.dumps(payload).encode())
            except asyncio.CancelledError:
                raise
            if proc.returncode:
                raise RuntimeError(f"MLX worker exited {proc.returncode}: {stderr.decode(errors='replace')[-1200:]}")
            output = json.loads(stdout.decode().splitlines()[-1])
            self.last_metrics = {"latency_seconds": round(time.monotonic() - started, 3),
                                 "peak_memory_gb": output.get("peak_memory_gb")}
            return ModelResponse(parts=[TextPart(output["text"])], model_name="Qwen3-VL-2B-Instruct-4bit")

        agent = Agent(FunctionModel(async_adapter), output_type=PydanticPromptedOutput(PromptedOutput), retries=0,
                      instructions="Use the supplied visual evidence and return the requested structured result.")
        result = await agent.run(prompt)
        parsed = result.output
        return Decision(observation_id=observation.observation_id,
                        outcome=parsed.outcome,
                        evidence_frame_ids=(selected.frame_id,),
                        explanation=parsed.explanation,
                        requested_view="closer view of the same canopy patch" if parsed.outcome == "inspect" else None)
