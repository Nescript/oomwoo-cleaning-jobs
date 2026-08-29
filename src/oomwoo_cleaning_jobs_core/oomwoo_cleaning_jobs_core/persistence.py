"""Local persistence of draft / published Region Sets."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import uuid
from typing import Callable

import cv2
import numpy as np
import yaml

from .constraints import ConstraintSet, Keepout, SpotArea, VirtualWall
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap

from .regions import RegionSet
from .validation import check_masks_overlap, validate_region_set

DEFAULT_STORAGE_ROOT = Path.home() / '.local/share/oomwoo_cleaning_jobs/maps'
SCHEMA_VERSION = 1


class PublishError(RuntimeError):
    """The published generation was written but a post-publish step failed;
    the published pointer has been rolled back to the previous generation."""


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
        keepout_margin_m: float = 0.0,
    ) -> Path:
        """Atomically replace the current map's draft; a draft may contain
        content that has not passed publish validation yet. The keepout mask
        files are materialized for preview only - Nav2 reads published only."""
        map_dir = self._map_dir(source_map)
        map_dir.mkdir(parents=True, exist_ok=True)
        self._write_snapshot(map_dir, source_map)
        target = map_dir / 'draft'
        self._write_set(target, source_map, region_set, constraints,
                        keepout_margin_m=keepout_margin_m)
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
        seed_pose: tuple[float, float] | None = None,
        keepout_margin_m: float = 0.0,
        post_publish_hook: Callable[[Path], None] | None = None,
    ) -> StoredRegionSet:
        """Save the draft after validation, then atomically replace the single
        published Region Set.

        ``seed_pose`` (dock staging pose, map frame) enables dock-relative
        reachability validation (enclosure / dock-trap detection).
        ``post_publish_hook`` runs after the published pointer switches
        (e.g. notifying the Nav2 keepout-mask publisher to reload); when it
        raises, the pointer is rolled back to the previous generation and a
        PublishError is raised, so disk and Nav2 never diverge silently.
        """
        keepout_mask = constraints.mask_for(source_map, keepout_margin_m)
        wall_band_mask = ConstraintSet(
            virtual_walls=constraints.virtual_walls).mask_for(source_map)
        report = validate_region_set(
            region_set, robot_inscribed_radius, keepout_mask,
            seed_pose=seed_pose, wall_band_mask=wall_band_mask)
        if not report.ok:
            codes = ', '.join(issue.code for issue in report.errors)
            raise ValueError(f'cannot publish Region Set: {codes}')

        self.save_draft(source_map, region_set, constraints, keepout_margin_m)
        map_dir = self._map_dir(source_map)
        target = map_dir / 'published'
        prior = self._load_set(target, source_map) if target.is_dir() else None
        version = 1 if prior is None or prior.version is None else prior.version + 1
        published_at = datetime.now(timezone.utc).isoformat()
        previous_generation = target.resolve() if target.is_symlink() else None
        self._write_set(target, source_map, region_set, constraints, version, published_at,
                        robot_inscribed_radius, keepout_margin_m, seed_pose,
                        prune_old_generation=post_publish_hook is None)
        if post_publish_hook is not None:
            try:
                post_publish_hook(target)
            except Exception as error:
                _restore_generation_pointer(target, previous_generation)
                raise PublishError(
                    f'post-publish step failed, published set rolled back: {error}') from error
            generations = target.parent / '.generations'
            if (previous_generation is not None
                    and previous_generation.parent == generations):
                shutil.rmtree(previous_generation, ignore_errors=True)
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
        keepout_margin_m: float = 0.0,
        seed_pose: tuple[float, float] | None = None,
        prune_old_generation: bool = True,
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
                metadata['keepout_margin_m'] = keepout_margin_m
                if seed_pose is not None:
                    metadata['seed_pose'] = [float(seed_pose[0]), float(seed_pose[1])]
            _atomic_yaml_dump(temp / 'regions.yaml', metadata)
            _atomic_yaml_dump(temp / 'constraints.yaml', _encode_constraints(constraints))
            _write_keepout_mask(temp, source_map,
                                constraints.mask_for(source_map, keepout_margin_m))
            _replace_active_generation(temp, target, prune_old=prune_old_generation)
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
    data = {
        'schema_version': SCHEMA_VERSION,
        'keepouts': [{'identifier': k.identifier, 'vertices': [list(p) for p in k.vertices]}
                     for k in constraints.keepouts],
        'virtual_walls': [{'identifier': w.identifier, 'start': list(w.start),
                           'end': list(w.end), 'width_m': w.width_m}
                          for w in constraints.virtual_walls],
    }
    if constraints.spot_area is not None:
        data['spot_area'] = {
            'identifier': constraints.spot_area.identifier,
            'vertices': [list(p) for p in constraints.spot_area.vertices],
            'name': constraints.spot_area.name,
        }
    return data


def _decode_constraints(data: dict) -> ConstraintSet:
    if data.get('schema_version') != SCHEMA_VERSION:
        raise ValueError('unsupported constraints schema_version')
    spot_data = data.get('spot_area')
    spot_area = None
    if spot_data is not None:
        spot_area = SpotArea(
            identifier=spot_data['identifier'],
            vertices=tuple(tuple(p) for p in spot_data['vertices']),
            name=spot_data.get('name', 'Spot Area'),
        )
    return ConstraintSet(
        keepouts=tuple(Keepout(item['identifier'], tuple(tuple(p) for p in item['vertices']))
                       for item in data.get('keepouts', [])),
        virtual_walls=tuple(VirtualWall(item['identifier'], tuple(item['start']),
                                         tuple(item['end']), item['width_m'])
                            for item in data.get('virtual_walls', [])),
        spot_area=spot_area,
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


def _write_keepout_mask(directory: Path, source_map: SourceMap, keepout_mask: np.ndarray) -> None:
    """Materialize the keepout mask as a Nav2-compatible trinary map
    (``keepout_mask.pgm`` + ``keepout_mask.yaml``) inside a generation dir.

    Constraint cells become black pixels (occ = 1.0 -> 100, LETHAL for the
    Nav2 KeepoutFilter); all other cells become white (free). The grid shares
    the Source Map geometry, so resolution and origin (including yaw) are
    copied verbatim and the image is vertically flipped like any map_saver
    PGM. The enclosed area behind Virtual Walls is deliberately NOT marked:
    it is unreachable by navigation topology, not an explicit keepout.
    """
    image_path = directory / 'keepout_mask.pgm'
    pixels = np.full(keepout_mask.shape, 255, dtype=np.uint8)
    pixels[keepout_mask] = 0
    if not cv2.imwrite(str(image_path), pixels[::-1, :]):
        raise OSError(f'failed to write keepout mask {image_path}')
    metadata = {
        'image': image_path.name,
        'resolution': source_map.resolution,
        'origin': list(source_map.origin), 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196, 'mode': 'trinary',
    }
    _atomic_yaml_dump(directory / 'keepout_mask.yaml', metadata)


def _restore_generation_pointer(target: Path, previous_generation: Path | None) -> None:
    """Roll back an atomic generation switch after a failed post-publish step."""
    current = target.resolve() if target.is_symlink() else None
    if previous_generation is not None and previous_generation.is_dir():
        pointer = target.parent / f'.{target.name}-pointer-{uuid.uuid4().hex}'
        os.symlink(os.path.relpath(previous_generation, target.parent), pointer)
        os.replace(pointer, target)
    elif target.is_symlink():
        target.unlink()
    if current is not None and current != previous_generation and current.is_dir():
        shutil.rmtree(current, ignore_errors=True)


def _replace_active_generation(temp: Path, target: Path, prune_old: bool = True) -> None:
    """Switch immutable generations via an atomic symlink pointer, so a crash
    always leaves one usable version. With prune_old=False the previous
    generation is kept so the caller can roll the pointer back."""
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
    if prune_old and old_generation is not None and old_generation.parent == generations:
        shutil.rmtree(old_generation, ignore_errors=True)
