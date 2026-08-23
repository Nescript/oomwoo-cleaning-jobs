"""Host-neutral editor controller composed from core APIs."""
from __future__ import annotations

import threading

import numpy as np
from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, VirtualWall
from oomwoo_cleaning_jobs_core.persistence import RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet
from oomwoo_cleaning_jobs_core.validation import validate_region_set
from oomwoo_segmentation.client import segment_once

class EditorController:
    def __init__(self, store=None, segmenter=None):
        self.store = store or RegionSetStore()
        # Injected in tests; production uses the provider-neutral ROS 2 action.
        self.segmenter = segmenter or segment_once
        self.source = None
        self.constraints = ConstraintSet()
        self.regions = None
        self._state_lock = threading.RLock()
        self._state_revision = 0

    def set_source(self, source):
        with self._state_lock:
            self._state_revision += 1
            self.source = source
            loaded = self.store.load_draft(source)
            if loaded:
                self.regions, self.constraints = loaded.region_set, loaded.constraints
                return 'Loaded draft for this map'
            self.regions = None
            self.constraints = ConstraintSet()
            others = self.store.other_map_set_count(source)
            return (f'No region set for the current map; {others} region set(s) on disk '
                    f'belong to other maps; please generate candidate regions')

    def generate_candidates(self):
        with self._state_lock:
            self._require_source()
            source = self.source
            source_identity = source.identity
            state_revision = self._state_revision
            keepout = self.constraints.mask_for(source)
            cleanable = source.free_mask() & ~keepout

        result = self.segmenter(source, cleanable_mask=cleanable)
        candidate_regions = RegionSet.from_segmentation(
            result,
            source.resolution,
            source.origin,
            base_cleanable=source.free_mask(),
            keepout_mask=keepout,
        )

        with self._state_lock:
            if self.source is None or self.source.identity != source_identity:
                raise RuntimeError(
                    'Source map changed while segmentation was running; '
                    'discarded stale result')
            if self._state_revision != state_revision:
                raise RuntimeError(
                    'Editor state changed while segmentation was running; '
                    'discarded stale result')
            self.regions = candidate_regions
        return result

    def paint_cell(self, label, row, col, erase=False):
        self._require_regions()
        stroke = np.zeros(self.source.cells.shape, dtype=bool)
        if 0 <= row < self.source.height and 0 <= col < self.source.width:
            stroke[row, col] = True
        if erase:
            return self.regions.erase(label, stroke), 'Erased'
        before = self.regions.labels.copy()
        changed = self.regions.paint(label, stroke)
        preempted = bool(changed and np.any((before != 0) & stroke & (before != label)))
        return changed, ('Preempted cells from another Region' if preempted
                         else ('Painted' if changed else 'Stroke lies entirely in non-cleanable space'))

    def create_rectangle(self, start_row, start_col, end_row, end_col, name):
        """Create a named Region from two diagonal cells; the core clips and preempts."""
        self._require_source()
        if self.regions is None:
            self.generate_candidates()
        row0, row1 = sorted((int(start_row), int(end_row)))
        col0, col1 = sorted((int(start_col), int(end_col)))
        stroke = np.zeros(self.source.cells.shape, dtype=bool)
        stroke[row0:row1 + 1, col0:col1 + 1] = True
        label = self.regions.create(stroke, name)
        return label, ('Region created' if label is not None
                       else 'Rectangle lies entirely in non-cleanable space')

    def add_keepout(self, identifier, vertices):
        """Add a map-frame polygon and immediately clip existing Region cells."""
        self._require_source()
        constraint = Keepout(identifier, tuple(vertices))
        self._set_constraints(ConstraintSet(
            keepouts=self.constraints.keepouts + (constraint,),
            virtual_walls=self.constraints.virtual_walls))

    def add_virtual_wall(self, identifier, start, end, width_m):
        """Add an explicit-width map-frame wall and immediately clip Regions."""
        self._require_source()
        constraint = VirtualWall(identifier, start, end, width_m)
        self._set_constraints(ConstraintSet(
            keepouts=self.constraints.keepouts,
            virtual_walls=self.constraints.virtual_walls + (constraint,)))

    def remove_constraint(self, identifier):
        self._require_source()
        keepouts = tuple(item for item in self.constraints.keepouts if item.identifier != identifier)
        walls = tuple(item for item in self.constraints.virtual_walls if item.identifier != identifier)
        if len(keepouts) == len(self.constraints.keepouts) and len(walls) == len(self.constraints.virtual_walls):
            raise ValueError(f'Constraint {identifier!r} does not exist')
        self._set_constraints(ConstraintSet(keepouts, walls))

    def _set_constraints(self, constraints):
        with self._state_lock:
            self._state_revision += 1
            self.constraints = constraints
            if self.regions is not None:
                self.regions.apply_keepout_mask(constraints.mask_for(self.source))

    def save_draft(self):
        self._require_regions(); return self.store.save_draft(self.source, self.regions, self.constraints)
    def report(self):
        self._require_regions(); return validate_region_set(self.regions, keepout_mask=self.constraints.mask_for(self.source))
    def publish(self):
        self._require_regions(); return self.store.publish(self.source, self.regions, self.constraints)
    def _require_source(self):
        if self.source is None: raise ValueError('Open a map first')
    def _require_regions(self):
        self._require_source()
        if self.regions is None: raise ValueError('Generate candidate regions first')
