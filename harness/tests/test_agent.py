import asyncio

import pytest

from ringquad_harness.agent import ReviewRunner, validate_decision
from ringquad_harness.schemas import Decision, EvidenceFrame, Observation


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
