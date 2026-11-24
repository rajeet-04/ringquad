from __future__ import annotations

import json
from pathlib import Path

from .schemas import SessionEvent


class SessionWriter:
    def __init__(self, output_dir: Path, *, config: dict | None = None):
        self.output_dir = output_dir
        self.config = config or {}
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output_dir / "events.jsonl"
        if self.events_path.exists() or (self.output_dir / "manifest.json").exists():
            raise FileExistsError(f"session output already contains data: {output_dir}")
        self._events = self.events_path.open("x", encoding="utf-8")
        self.stats = {"decoded": 0, "processed": 0, "skipped": 0, "dropped": 0}

    def write_event(self, event: SessionEvent) -> None:
        self._events.write(event.model_dump_json() + "\n")
        self._events.flush()

    def close(self) -> None:
        if not self._events.closed:
            self._events.close()

    def finalize(self, *, session_id: str, video: str, stage: str) -> None:
        self.close()
        manifest = {
            "session_id": session_id,
            "video": video,
            "stage": stage,
            "configuration": self.config,
            "statistics": self.stats,
            "events": self.events_path.name,
        }
        (self.output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    def __enter__(self) -> "SessionWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
