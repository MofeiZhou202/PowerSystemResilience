from __future__ import annotations

import json
import unittest
from typing import Any, Mapping

from hysim import (
    ApiError,
    HySimV1Session,
    HySimV1Client,
    HySimV1ToolRegistry,
    JobFailedError,
    PowerFlowOptions,
    PowerFlowRequest,
    TransportResponse,
    ToolEffect,
    ToolPolicy,
    ToolPolicyError,
)
from hysim.v1 import HySimJob


class FakeTransport:
    def __init__(
        self,
        responses: list[tuple[int, Mapping[str, str], Mapping[str, Any]]],
    ) -> None:
        self.responses = list(responses)
        self.calls: list[dict[str, Any]] = []

    def request(self, method: str, path: str, **kwargs: Any) -> TransportResponse:
        self.calls.append({"method": method, "path": path, **kwargs})
        status, headers, body = self.responses.pop(0)
        return TransportResponse(status, headers, json.dumps(body).encode())


def session_body(revision: int) -> dict[str, Any]:
    return {
        "schema": "hysim_session_v1",
        "session_id": "ses-1",
        "model_revision": revision,
        "etag": f'"hysim-ses-1-r{revision}"',
        "has_model": True,
    }


class V1ClientTests(unittest.TestCase):
    def test_session_tracks_etag_across_replace_and_submit(self) -> None:
        transport = FakeTransport(
            [
                (201, {"ETag": '"hysim-ses-1-r1"'}, session_body(1)),
                (200, {"ETag": '"hysim-ses-1-r2"'}, session_body(2)),
                (
                    202,
                    {},
                    {
                        "schema": "hysim_job_v1",
                        "job_id": "job-1",
                        "session_id": "ses-1",
                        "model_revision": 2,
                        "analysis": "power_flow",
                        "state": "queued",
                    },
                ),
            ]
        )
        client = HySimV1Client(transport=transport)

        session = client.create_session(case="ieee14_acdc")
        session.replace_model(case="case9")
        job = session.power_flow(
            PowerFlowRequest(options=PowerFlowOptions(max_iter=50, tol=1e-8))
        )

        self.assertEqual(session.model_revision, 2)
        self.assertEqual(job.job_id, "job-1")
        self.assertEqual(
            transport.calls[1]["headers"]["If-Match"],
            '"hysim-ses-1-r1"',
        )
        self.assertEqual(
            transport.calls[2]["headers"]["If-Match"],
            '"hysim-ses-1-r2"',
        )
        self.assertEqual(
            transport.calls[2]["json_body"]["request"]["options"]["max_iter"],
            50,
        )

    def test_job_wait_and_result_preserve_revision_contract(self) -> None:
        transport = FakeTransport(
            [
                (
                    200,
                    {},
                    {
                        "job_id": "job-2",
                        "session_id": "ses-1",
                        "model_revision": 3,
                        "analysis": "power_flow",
                        "state": "running",
                    },
                ),
                (
                    200,
                    {},
                    {
                        "job_id": "job-2",
                        "session_id": "ses-1",
                        "model_revision": 3,
                        "current_model_revision": 4,
                        "analysis": "power_flow",
                        "state": "succeeded",
                        "result": {
                            "converged": True,
                            "iterations": 4,
                            "_result_contract": {
                                "schema": "hysim_result_v1",
                                "stale": True,
                            },
                        },
                    },
                ),
            ]
        )
        job = HySimV1Client(transport=transport).get_job("job-2")
        job.wait(timeout=1.0, poll_interval=0.0)
        result = job.result()

        self.assertEqual(result.model_revision, 3)
        self.assertTrue(result.stale)
        self.assertEqual(result.scientific_status, "stale")

    def test_revision_conflict_surfaces_as_precondition_error(self) -> None:
        transport = FakeTransport(
            [
                (201, {"ETag": '"hysim-ses-1-r1"'}, session_body(1)),
                (
                    412,
                    {"ETag": '"hysim-ses-1-r2"'},
                    {
                        "error": "revision_conflict",
                        "message": "ETag does not match",
                    },
                ),
            ]
        )
        session = HySimV1Client(transport=transport).create_session(case="case9")
        with self.assertRaises(ApiError) as caught:
            session.replace_model(case="ieee14_acdc")
        self.assertEqual(caught.exception.status_code, 412)

    def test_failed_job_does_not_expose_a_result(self) -> None:
        transport = FakeTransport(
            [
                (
                    200,
                    {},
                    {
                        "job_id": "job-3",
                        "session_id": "ses-1",
                        "model_revision": 1,
                        "analysis": "power_flow",
                        "state": "failed",
                        "error": {"message": "unsupported method"},
                    },
                )
            ]
        )
        job = HySimV1Client(transport=transport).get_job("job-3")
        with self.assertRaises(JobFailedError):
            job.result()

    def test_lod_subgraph_and_frame_queries_are_chunked(self) -> None:
        transport = FakeTransport(
            [
                (200, {}, {"schema": "hysim_topology_chunk_v1", "nodes": []}),
                (200, {}, {"schema": "hysim_subgraph_v1", "nodes": []}),
                (200, {}, {"schema": "hysim_result_frame_chunk_v1", "nodes": []}),
                (200, {}, {"schema": "hysim_result_violation_chunk_v1", "items": []}),
            ]
        )
        client = HySimV1Client(transport=transport)
        session = HySimV1Session(
            client, "ses-1", 1, '"hysim-ses-1-r1"', session_body(1)
        )
        session.topology(lod=1, offset=20, limit=50, viewport=(0, 1, 2, 3))
        session.subgraph("dc", 7, depth=3, max_nodes=200)
        runtime_job = HySimJob(client, "job-1", {"state": "succeeded"})
        runtime_job.frame(4, domain="ac", limit=25, indices=(2, 9))
        runtime_job.violations(4, limit=10)

        self.assertEqual(transport.calls[0]["query"]["lod"], 1)
        self.assertEqual(transport.calls[0]["query"]["xmax"], 2)
        self.assertEqual(transport.calls[1]["query"]["domain"], "dc")
        self.assertEqual(transport.calls[1]["query"]["index"], 7)
        self.assertEqual(transport.calls[2]["path"], "/api/v1/jobs/job-1/frames/4")
        self.assertEqual(transport.calls[2]["query"]["indices"], "2,9")
        self.assertEqual(transport.calls[3]["query"]["step"], 4)

    def test_v1_ai_tools_submit_jobs_but_guard_model_changes(self) -> None:
        transport = FakeTransport(
            [
                (
                    202,
                    {},
                    {
                        "job_id": "job-ai",
                        "session_id": "ses-1",
                        "model_revision": 1,
                        "analysis": "power_flow",
                        "state": "queued",
                    },
                ),
                (200, {"ETag": '"hysim-ses-1-r2"'}, session_body(2)),
            ]
        )
        session = HySimV1Session(
            HySimV1Client(transport=transport),
            "ses-1",
            1,
            '"hysim-ses-1-r1"',
            session_body(1),
        )
        registry = HySimV1ToolRegistry(session)
        submitted = registry.call("hysim_v1_submit_power_flow", {"max_iter": 20})
        self.assertEqual(submitted["job_id"], "job-ai")
        with self.assertRaises(ToolPolicyError):
            registry.call(
                "hysim_v1_replace_builtin",
                {"case": "case9"},
                approved=True,
            )

        registry = HySimV1ToolRegistry(
            session,
            policy=ToolPolicy(
                allowed_effects=frozenset(
                    {ToolEffect.READ, ToolEffect.ANALYZE, ToolEffect.MODIFY_MODEL}
                )
            ),
        )
        changed = registry.call(
            "hysim_v1_replace_builtin",
            {"case": "case9"},
            approved=True,
        )
        self.assertEqual(changed["model_revision"], 2)


if __name__ == "__main__":
    unittest.main()
