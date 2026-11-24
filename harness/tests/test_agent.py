import asyncio
import json

import pytest

from ringquad_harness.agent import ReviewRunner, validate_decision
from ringquad_harness.schemas import Decision, EvidenceFrame, Observation
from ringquad_harness.evaluate import evaluate_manifest, review_session
from ringquad_harness.cli import replay
from ringquad_harness.model_mlx import _communicate_cancel_safe


def observation() -> Observation:
    ev = EvidenceFrame("session:camera:4", 40, "/tmp/context.jpg", "/tmp/crop.jpg")
    return Observation("patch-1", "session", "camera", 0, 40, (1, 2, 5, 6),
                       (ev.frame_id,), (ev,))


class Backend:
    def __init__(self, result=None, error=None, delay=0):
        self.result, self.error, self.delay = result, error, delay

    async def review(self, observation, evidence):
        await asyncio.sleep(self.delay)
        if self.error:
            raise self.error
        return self.result


def test_valid_inspect_decision_must_cite_observed_frame_and_request_view():
    obs = observation()
    decision = Decision(observation_id=obs.observation_id, outcome="inspect",
                        evidence_frame_ids=(obs.evidence_frame_ids[0],), explanation="Localized discoloration",
                        requested_view="closer view of same patch")
    assert validate_decision(decision, obs) == decision


def test_rejects_cross_track_evidence_even_if_decision_is_schema_valid():
    obs = observation()
    decision = Decision(observation_id=obs.observation_id, outcome="inspect",
                        evidence_frame_ids=("other:camera:8",), explanation="Looks unusual",
                        requested_view="closer view")
    with pytest.raises(ValueError, match="outside this observation"):
        validate_decision(decision, obs)


def test_timeout_is_recorded_as_failure_with_zero_proposals():
    async def run():
        obs = observation()
        runner = ReviewRunner(Backend(delay=0.05), timeout=0.001)
        worker = asyncio.create_task(runner.run())
        await runner.submit(obs)
        await runner.close()
        return await worker
    (obs, result), = asyncio.run(run())
    assert result.error and "timed out" in result.error
    assert result.proposals == 0


def test_invalid_output_is_recorded_as_failure_with_zero_proposals():
    async def run():
        obs = observation()
        invalid = Decision(observation_id=obs.observation_id, outcome="inspect",
                           evidence_frame_ids=(obs.evidence_frame_ids[0],),
                           explanation="uncertain", requested_view=None)
        runner = ReviewRunner(Backend(result=invalid))
        worker = asyncio.create_task(runner.run())
        await runner.submit(obs)
        await runner.close()
        return await worker
    (obs, result), = asyncio.run(run())
    assert result.error and "requested view" in result.error
    assert result.proposals == 0


def test_deduplicates_observation_id():
    async def run():
        obs = observation()
        decision = Decision(observation_id=obs.observation_id, outcome="monitor",
                            evidence_frame_ids=(obs.evidence_frame_ids[0],), explanation="No clear abnormality")
        runner = ReviewRunner(Backend(result=decision))
        worker = asyncio.create_task(runner.run())
        await runner.submit(obs)
        await runner.submit(obs)
        await runner.close()
        return await worker
    assert len(asyncio.run(run())) == 1


def test_results_keep_independent_metric_snapshots_and_empty_queue_has_zero_metrics():
    class MeasuringBackend:
        last_metrics = {}

        async def review(self, obs, evidence):
            self.last_metrics = {"latency_seconds": 1.0 if obs.observation_id == "first" else 2.0}
            return Decision(observation_id=obs.observation_id, outcome="monitor",
                            evidence_frame_ids=(obs.evidence_frame_ids[0],), explanation="Readable")

    async def run():
        backend = MeasuringBackend()
        runner = ReviewRunner(backend, queue_size=1)
        worker = asyncio.create_task(runner.run())
        for name in ("first", "second"):
            original = observation()
            obs = Observation(name, original.session_id, original.camera_id, original.first_capture_ns,
                              original.last_capture_ns, original.bbox, original.evidence_frame_ids,
                              original.evidence)
            await runner.submit(obs)
        await runner.close()
        results = await worker
        empty = ReviewRunner(backend)
        empty_worker = asyncio.create_task(empty.run())
        await empty.close()
        await empty_worker
        return results, empty

    results, empty = asyncio.run(run())
    assert [result.metrics for _, result in results] == [
        {"latency_seconds": 1.0}, {"latency_seconds": 2.0}
    ]
    assert empty.completed == empty.proposals == empty.failures == 0


def test_review_session_streams_individual_metrics_and_emits_portable_evidence(tmp_path):
    class MeasuringBackend:
        metadata = {"provider": "test", "revision": "fake-r1"}
        last_metrics = {}

        async def review(self, obs, evidence):
            self.last_metrics = {"latency_seconds": 1.0 if obs.observation_id == "first" else 2.0}
            return Decision(observation_id=obs.observation_id, outcome="inspect",
                            evidence_frame_ids=(evidence[0].frame_id,), explanation="Visible change",
                            requested_view="closer view")

    session = tmp_path / "session"
    session.mkdir()
    ev = observation().evidence[0]
    (session / "context.jpg").write_bytes(b"context")
    (session / "crop.jpg").write_bytes(b"crop")
    events = []
    for observation_id in ("first", "second"):
        obs = Observation(observation_id, "session", "camera", 0, 40, (1, 2, 5, 6),
                          (ev.frame_id,), (EvidenceFrame(ev.frame_id, ev.capture_ns, "context.jpg", "crop.jpg"),))
        events.append({"event_type": "observation", "timestamp_ns": 40, "session_id": "session",
                       "camera_id": "camera", "data": {**obs.__dict__,
                       "evidence_frame_ids": list(obs.evidence_frame_ids),
                       "evidence": [e.__dict__ for e in obs.evidence]}})
    (session / "events.jsonl").write_text("".join(json.dumps(e, default=str) + "\n" for e in events))
    (session / "manifest.json").write_text(json.dumps({"stage": "perception", "events": "events.jsonl"}))
    summary = asyncio.run(review_session(session, MeasuringBackend(), queue_size=1))
    review_rows = [json.loads(line)["data"] for line in (session / "events.jsonl").read_text().splitlines()
                   if json.loads(line)["event_type"] == "review"]
    assert summary["reviews"] == 2 and summary["inspection_proposals"] == 2
    assert [row["model_metrics"]["latency_seconds"] for row in review_rows] == [1.0, 2.0]
    assert review_rows[0]["evidence_refs"] == [{"frame_id": ev.frame_id, "capture_ns": ev.capture_ns,
        "context_path": "context.jpg", "crop_path": "crop.jpg"}]
    assert review_rows[0]["model"]["revision"] == "fake-r1"


def test_review_session_rejects_bundle_path_escape(tmp_path):
    class NeverCalled:
        async def review(self, observation, evidence):
            raise AssertionError("must reject before model call")
    session = tmp_path / "session"
    session.mkdir()
    ev = EvidenceFrame("session:camera:4", 4, "../outside.jpg", "crop.jpg")
    obs = observation()
    data = {**obs.__dict__, "evidence_frame_ids": list(obs.evidence_frame_ids), "evidence": [ev.__dict__]}
    (session / "events.jsonl").write_text(json.dumps({"event_type": "observation", "data": data}) + "\n")
    (session / "manifest.json").write_text(json.dumps({"stage": "perception", "events": "events.jsonl"}))
    with pytest.raises(ValueError, match="escapes"):
        asyncio.run(review_session(session, NeverCalled()))


def test_replay_review_rejects_bad_timeout_or_model_before_creating_output(tmp_path):
    out = tmp_path / "session"
    assert replay(tmp_path / "video.mp4", out, "review", model_path=tmp_path, review_timeout=float("inf")) == 2
    assert not out.exists()
    assert replay(tmp_path / "video.mp4", out, "review", model_path=tmp_path / "missing", review_timeout=1) == 2
    assert not out.exists()


def test_replay_review_returns_nonzero_and_keeps_failure_event(tmp_path, monkeypatch):
    import numpy as np
    import ringquad_harness.evaluate as evaluate_module
    import ringquad_harness.model_mlx as mlx_module
    import ringquad_harness.cli as cli_module
    from ringquad_harness.schemas import Frame, SessionEvent

    model = tmp_path / "model"
    model.mkdir()
    class FakeBackend:
        def __init__(self, model_path, timeout):
            self.model_path = model_path
    async def fail_review(session_dir, backend, timeout):
        with (session_dir / "events.jsonl").open("a") as stream:
            stream.write(SessionEvent(event_type="review", timestamp_ns=1, session_id="s",
                data={"status": "failed", "proposal_count": 0, "error": "timeout"}).model_dump_json() + "\n")
        return {"observations": 1, "reviews": 1, "inspection_proposals": 0, "failures": 1}

    monkeypatch.setattr(mlx_module, "MLXReviewBackend", FakeBackend)
    monkeypatch.setattr(evaluate_module, "review_session", fail_review)
    monkeypatch.setattr(cli_module, "iter_video", lambda *args, **kwargs: iter([
        Frame("s", "camera", 0, 0, "video_pts_relative", np.zeros((8, 8, 3), dtype=np.uint8))
    ]))
    output = tmp_path / "reviewed"
    assert replay(tmp_path / "fixture.mp4", output, "review", model_path=model, review_timeout=3) == 1
    events = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    assert any(event["event_type"] == "review" and event["data"]["status"] == "failed" for event in events)


def test_evaluation_manifest_requires_exact_expected_observation_schema(tmp_path):
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"clips": [{"clip_id": "case-a", "path": "a.mp4",
                                               "expected_review_outcome": "monitor"}]}))
    with pytest.raises(ValueError, match="expected_observations"):
        evaluate_manifest(manifest, tmp_path / "output", tmp_path, Backend())
    assert not (tmp_path / "output").exists()


def test_cancelled_model_worker_closes_stdin_drains_pipes_and_reaps():
    class FakeStdin:
        closed = False
        def is_closing(self):
            return self.closed
        def close(self):
            self.closed = True

    class FakeProcess:
        def __init__(self, lookup_error=False):
            self.stdin = FakeStdin()
            self.entered = asyncio.Event()
            self.finished = asyncio.Event()
            self.drained = False
            self.lookup_error = lookup_error
        async def communicate(self, payload):
            self.entered.set()
            await self.finished.wait()
            self.drained = True
            return b"out", b"err"
        def kill(self):
            self.finished.set()
            if self.lookup_error:
                raise ProcessLookupError
        async def wait(self):
            return 0

    async def run(lookup_error):
        proc = FakeProcess(lookup_error)
        task = asyncio.create_task(_communicate_cancel_safe(proc, b"payload"))
        await proc.entered.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        return proc

    normal = asyncio.run(run(False))
    gone = asyncio.run(run(True))
    assert normal.stdin.closed and normal.drained
    assert gone.stdin.closed and gone.drained
