"""Observations YAML: capture records + run metadata (02-features §D5).

``schema_version`` is an integer bumped on any incompatible change; the loader
refuses versions it doesn't know.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .boards import BoardSpec
from .capture import OBSERVATIONS_SCHEMA_VERSION, CaptureRecord


class ObservationsError(ValueError):
    pass


MODES = ('eye_to_hand', 'eye_in_hand')
FRAME_KEYS = ('base_frame', 'tool_frame', 'camera_frame')


def check_metadata(metadata: dict[str, Any]) -> None:
    """§D5 run metadata: mode and frame names are required."""
    if metadata.get('mode') not in MODES:
        raise ObservationsError(f'metadata.mode must be one of {MODES}')
    frames = metadata.get('frames')
    if not isinstance(frames, dict) or any(
            not isinstance(frames.get(k), str) or not frames.get(k) for k in FRAME_KEYS):
        raise ObservationsError(f'metadata.frames must name {", ".join(FRAME_KEYS)}')


def dump(records: list[CaptureRecord], board: BoardSpec, metadata: dict[str, Any]) -> str:
    check_metadata(metadata)
    doc = {
        'schema_version': OBSERVATIONS_SCHEMA_VERSION,
        'metadata': dict(metadata),
        'board': board.to_dict(),
        'captures': [r.to_dict() for r in records],
    }
    return yaml.safe_dump(doc, sort_keys=False)


def load(text: str) -> tuple[list[CaptureRecord], BoardSpec, dict[str, Any]]:
    doc = yaml.safe_load(text)
    if not isinstance(doc, dict) or 'schema_version' not in doc:
        raise ObservationsError('not an observations file (no schema_version)')
    version = doc['schema_version']
    if type(version) is not int or version != OBSERVATIONS_SCHEMA_VERSION:
        raise ObservationsError(
            f'observations schema_version {version!r} is not supported '
            f'(this version reads the integer {OBSERVATIONS_SCHEMA_VERSION})')
    metadata = dict(doc.get('metadata') or {})
    check_metadata(metadata)
    try:
        records = [CaptureRecord.from_dict(c) for c in doc.get('captures') or []]
    except (KeyError, ValueError) as e:
        raise ObservationsError(f'bad capture record: {e}') from e
    return records, BoardSpec.from_dict(doc['board']), metadata


def save(path: str | Path, records: list[CaptureRecord], board: BoardSpec,
         metadata: dict[str, Any]) -> None:
    Path(path).write_text(dump(records, board, metadata))


def read(path: str | Path) -> tuple[list[CaptureRecord], BoardSpec, dict[str, Any]]:
    return load(Path(path).read_text())
