"""Local persistence of draft / published Region Sets."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import uuid

import cv2
import numpy as np
import yaml

from .constraints import ConstraintSet, Keepout, VirtualWall
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap

from .regions import RegionSet
from .validation import check_masks_overlap, validate_region_set

DEFAULT_STORAGE_ROOT = Path.home() / '.local/share/oomwoo_cleaning_jobs/maps'
SCHEMA_VERSION = 1


@dataclass(frozen=True)
class StoredRegionSet:
    region_set: RegionSet
    constraints: ConstraintSet
    version: int | None = None
    published_at: str | None = None


class RegionSetStore:
    """Draft/published Region Set storage partitioned by Source Map identity."""

    def __init__(self, root: str | os.PathLike = DEFAULT_STORAGE_ROOT) -> None:
        self.root = Path(root)

    def save_draft(
        self, source_map: SourceMap, region_set: RegionSet,
        constraints: ConstraintSet,
    ) -> Path:
        """Atomically replace the current map's draft; a draft may contain
        content that has not passed publish validation yet."""
        map_dir = self._map_dir(source_map)
        map_dir.mkdir(parents=True, exist_ok=True)
        self._write_snapshot(map_dir, source_map)
        target = map_dir / 'draft'
        self._write_set(target, source_map, region_set, constraints)
        return target

    def load_draft(self, source_map: SourceMap) -> StoredRegionSet | None:
        target = self._map_dir(source_map) / 'draft'
        return None if not target.is_dir() else self._load_set(target, source_map)

    def other_map_set_count(self, source_map: SourceMap) -> int:
        """Count other map identities with an active draft or published Region Set."""
        if not self.root.is_dir():
            return 0
        return sum(
            1 for directory in self.root.iterdir()
            if directory.is_dir() and directory.name != source_map.identity
            and ((directory / 'draft').is_dir() or (directory / 'published').is_dir())
        )

    def publish(
        self, source_map: SourceMap, region_set: RegionSet,
        constraints: ConstraintSet,
        robot_inscribed_radius: float = 0.17,
    ) -> StoredRegionSet:
        """Save the draft after validation, then atomically replace the single
        published Region Set."""
        keepout_mask = constraints.mask_for(source_map)
        report = validate_region_set(region_set, robot_inscribed_radius, keepout_mask)
        if not report.ok:
            codes = ', '.join(issue.code for issue in report.errors)
            raise ValueError(f'cannot publish Region Set: {codes}')

        self.save_draft(source_map, region_set, constraints)
        map_dir = self._map_dir(source_map)
        target = map_dir / 'published'
        prior = self._load_set(target, source_map) if target.is_dir() else None
        version = 1 if prior is None or prior.version is None else prior.version + 1
        published_at = datetime.now(timezone.utc).isoformat()
        self._write_set(target, source_map, region_set, constraints, version, published_at,
                        robot_inscribed_radius)
        return StoredRegionSet(region_set, constraints, version, published_at)

    def load_published(self, source_map: SourceMap) -> StoredRegionSet | None:
        target = self._map_dir(source_map) / 'published'
        return None if not target.is_dir() else self._load_set(target, source_map, require_valid=True)

    def _map_dir(self, source_map: SourceMap) -> Path:
        return self.root / source_map.identity

    def _write_snapshot(self, map_dir: Path, source_map: SourceMap) -> None:
        yaml_path = map_dir / 'map_snapshot.yaml'
        image_path = map_dir / 'map_snapshot.pgm'
        raw_path = map_dir / 'map_snapshot.cells.npy'
        if yaml_path.exists() and image_path.exists() and raw_path.exists():
            return
        # The PGM is a nav2 trinary preview for human inspection; the raw
        # sidecar is the lossless provenance record.
        raw_temp = raw_path.with_name(f'.{raw_path.name}-{uuid.uuid4().hex}')
        with raw_temp.open('wb') as stream:
            np.save(stream, source_map.cells)
        os.replace(raw_temp, raw_path)
        pixels = np.full(source_map.cells.shape, 205, dtype=np.uint8)
        pixels[source_map.cells == FREE] = 254
        pixels[source_map.cells >= OCCUPIED] = 0
        if not cv2.imwrite(str(image_path), pixels[::-1, :]):
            raise OSError(f'failed to write map snapshot {image_path}')
        metadata = {
            'image': image_path.name, 'raw_cells': raw_path.name,
            'resolution': source_map.resolution,
            'origin': list(source_map.origin), 'negate': 0,
            'occupied_thresh': 0.65, 'free_thresh': 0.196, 'mode': 'trinary',
        }
        _atomic_yaml_dump(yaml_path, metadata)

    def _write_set(
        self, target: Path, source_map: SourceMap, region_set: RegionSet,
        constraints: ConstraintSet, version: int | None = None,
        published_at: str | None = None,
        robot_inscribed_radius: float | None = None,
    ) -> None:
        if region_set.labels.shape != source_map.cells.shape:
            raise ValueError('RegionSet and SourceMap grid shapes differ')
        temp = target.parent / f'.{target.name}-{uuid.uuid4().hex}'
        temp.mkdir(parents=True)
        try:
            masks_dir = temp / 'masks'
            masks_dir.mkdir()
            regions = []
            for info in region_set.regions():
                mask_path = masks_dir / f'{info.label}.png'
                # PNGs are written in usual image orientation; loading restores
                # the SourceMap convention row 0 = bottom row.
                if not cv2.imwrite(str(mask_path), (region_set.mask_of(info.label)[::-1] * 255).astype(np.uint8)):
                    raise OSError(f'failed to write Region mask {mask_path}')
                regions.append({'label': info.label, 'name': info.name, 'mask': f'masks/{info.label}.png'})
            metadata = {'schema_version': SCHEMA_VERSION, 'map_identity': source_map.identity,
                        'regions': regions}
            if version is not None:
                metadata['version'] = version
                metadata['published_at'] = published_at
                metadata['robot_inscribed_radius'] = robot_inscribed_radius
            _atomic_yaml_dump(temp / 'regions.yaml', metadata)
            _atomic_yaml_dump(temp / 'constraints.yaml', _encode_constraints(constraints))
            _replace_active_generation(temp, target)
        except BaseException:
            shutil.rmtree(temp, ignore_errors=True)
            raise

    def _load_set(
        self, target: Path, source_map: SourceMap, require_valid: bool = False,
    ) -> StoredRegionSet:
        meta = _load_yaml(target / 'regions.yaml')
        if meta.get('schema_version') != SCHEMA_VERSION:
            raise ValueError(f'{target}: unsupported Region Set schema_version')
        if meta.get('map_identity') != source_map.identity:
            raise ValueError(f'{target}: Source Map identity mismatch')
        constraints = _decode_constraints(_load_yaml(target / 'constraints.yaml'))
        masks: dict[int, np.ndarray] = {}
        names: dict[int, str] = {}
        for region in meta.get('regions', []):
            label = int(region['label'])
            if label <= 0 or label in masks:
                raise ValueError(f'{target}: Region label invalid or duplicated')
            relative_mask = Path(region['mask'])
            expected_mask = Path('masks') / f'{label}.png'
            if relative_mask != expected_mask or relative_mask.is_absolute():
                raise ValueError(f'{target}: invalid Region mask path')
            image = cv2.imread(str(target / relative_mask), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f'{target}: failed to read Region mask')
            mask = image[::-1, :] > 0
            if mask.shape != source_map.cells.shape:
                raise ValueError(f'{target}: Region mask shape mismatch')
            masks[label] = mask
            names[label] = str(region['name'])
        overlap = check_masks_overlap(masks)
        if overlap:
            raise ValueError(f'{target}: Region masks overlap')
        labels = np.zeros(source_map.cells.shape, dtype=np.int32)
        for label, mask in masks.items():
            labels[mask] = label
        keepout_mask = constraints.mask_for(source_map)
        region_set = RegionSet(labels, source_map.free_mask() & ~keepout_mask,
                               source_map.resolution, source_map.origin, names,
                               base_cleanable=source_map.free_mask())
        # Keep constraint intersections from hand-edited files so publish
        # validation can report the invariant violation, instead of silently
        # clipping them away at load time.
        region_set.keepout_mask = keepout_mask
        if require_valid:
            radius = meta.get('robot_inscribed_radius')
            if not isinstance(radius, (int, float)) or radius <= 0:
                raise ValueError(f'{target}: Published Region Set lacks a valid footprint radius')
            report = validate_region_set(region_set, float(radius), keepout_mask)
            if not report.ok:
                codes = ', '.join(issue.code for issue in report.errors)
                raise ValueError(f'{target}: Published Region Set failed validation: {codes}')
        return StoredRegionSet(region_set, constraints, meta.get('version'), meta.get('published_at'))


def _encode_constraints(constraints: ConstraintSet) -> dict:
    return {
        'schema_version': SCHEMA_VERSION,
        'keepouts': [{'identifier': k.identifier, 'vertices': [list(p) for p in k.vertices]}
                     for k in constraints.keepouts],
        'virtual_walls': [{'identifier': w.identifier, 'start': list(w.start),
                           'end': list(w.end), 'width_m': w.width_m}
                          for w in constraints.virtual_walls],
    }


def _decode_constraints(data: dict) -> ConstraintSet:
    if data.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('unsupported constraints schema_version')
    return ConstraintSet(
        keepouts=tuple(Keepout(item['identifier'], tuple(tuple(p) for p in item['vertices']))
                       for item in data.get('keepouts', [])),
        virtual_walls=tuple(VirtualWall(item['identifier'], tuple(item['start']),
                                         tuple(item['end']), item['width_m'])
                            for item in data.get('virtual_walls', [])),
    )


def _load_yaml(path: Path) -> dict:
    try:
        with path.open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'failed to read valid YAML: {path}') from error
    if not isinstance(data, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    return data


def _atomic_yaml_dump(path: Path, data: dict) -> None:
    temp = path.with_name(f'.{path.name}-{uuid.uuid4().hex}')
    with temp.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
    os.replace(temp, path)


def _replace_active_generation(temp: Path, target: Path) -> None:
    """Switch immutable generations via an atomic symlink pointer, so a crash
    always leaves one usable version."""
    generations = target.parent / '.generations'
    generations.mkdir(exist_ok=True)
    generation = generations / f'{target.name}-{uuid.uuid4().hex}'
    os.replace(temp, generation)

    # A legacy implementation may have left a plain directory; on first
    # migration, keep it as a generation before installing the pointer.
    if target.exists() and not target.is_symlink():
        legacy = generations / f'{target.name}-legacy-{uuid.uuid4().hex}'
        os.replace(target, legacy)
    old_generation = target.resolve() if target.is_symlink() else None
    pointer = target.parent / f'.{target.name}-pointer-{uuid.uuid4().hex}'
    os.symlink(os.path.relpath(generation, target.parent), pointer)
    os.replace(pointer, target)
    if old_generation is not None and old_generation.parent == generations:
        shutil.rmtree(old_generation, ignore_errors=True)
