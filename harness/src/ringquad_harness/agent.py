from __future__ import annotations

import asyncio
from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Callable, Protocol, Sequence

from .schemas import Decision, EvidenceFrame, Observation


class ReviewBackend(Protocol):
    async def review(self, observation: Observation, evidence: Sequence[EvidenceFrame]) -> Decision: ...


def validate_decision(decision: Decision, observation: Observation) -> Decision:
    """Enforce the observation and evidence boundary independently of model schema."""
    if decision.observation_id != observation.observation_id:
        raise ValueError("decision references a different observation")
    allowed = set(observation.evidence_frame_ids)
    if not allowed or not set(decision.evidence_frame_ids).issubset(allowed):
        raise ValueError("decision cites evidence outside this observation")
    if decision.outcome not in {"inspect", "monitor", "insufficient_evidence"}:
        raise ValueError("unsupported review outcome")
    if decision.outcome == "inspect":
        if not decision.evidence_frame_ids:
            raise ValueError("inspect requires observed evidence frames")
        if not decision.requested_view or not decision.requested_view.strip():
            raise ValueError("inspect requires a requested view")
    elif decision.requested_view is not None:
        raise ValueError("only inspect may request a view")
    return decision


@dataclass(frozen=True)
class ReviewResult:
    decision: Decision | None
    error: str | None = None
    metrics: dict | None = None

    @property
    def proposals(self) -> int:
        return int(self.decision is not None and self.decision.outcome == "inspect")


class ReviewRunner:
    """One bounded FIFO review queue; tracks are accepted at most once."""

    def __init__(self, backend: ReviewBackend, *, timeout: float = 60.0, queue_size: int = 1,
                 on_result: Callable[[Observation, ReviewResult], None] | None = None,
                 collect_results: bool = True):
        if queue_size < 1 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("queue_size and timeout must be positive")
        self.backend, self.timeout = backend, timeout
        self.on_result, self.collect_results = on_result, collect_results
        self.queue: asyncio.Queue[tuple[Observation, Sequence[EvidenceFrame]] | None] = asyncio.Queue(queue_size)
        self._seen: set[str] = set()
        self.results: list[tuple[Observation, ReviewResult]] = []
        self.proposals = self.failures = self.completed = 0

    async def submit(self, observation: Observation, evidence: Sequence[EvidenceFrame] | None = None) -> None:
        if observation.observation_id in self._seen:
            return
        self._seen.add(observation.observation_id)
        await self.queue.put((observation, tuple(evidence if evidence is not None else observation.evidence)))

    async def run(self) -> list[tuple[Observation, ReviewResult]]:
        while True:
            item = await self.queue.get()
            if item is None:
                self.queue.task_done()
                break
            observation, evidence = item
            try:
                decision = await asyncio.wait_for(self.backend.review(observation, evidence), self.timeout)
                validate_decision(decision, observation)
                result = ReviewResult(decision)
            except asyncio.TimeoutError:
                result = ReviewResult(None, f"review timed out after {self.timeout:g}s")
            except Exception as exc:
                result = ReviewResult(None, f"review failed: {type(exc).__name__}: {exc}")
            result = ReviewResult(result.decision, result.error,
                                  deepcopy(getattr(self.backend, "last_metrics", {}) or {}))
            self.completed += 1
            self.proposals += result.proposals
            self.failures += result.error is not None
            if self.on_result is not None:
                self.on_result(observation, result)
            if self.collect_results:
                self.results.append((observation, result))
            self.queue.task_done()
        return self.results

    async def close(self) -> None:
        await self.queue.put(None)
