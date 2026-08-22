"""Host-neutral editor controller composed from core APIs."""
from __future__ import annotations
import numpy as np
from oomwoo_cleaning_jobs_core.constraints import ConstraintSet, Keepout, VirtualWall
from oomwoo_cleaning_jobs_core.persistence import RegionSetStore
from oomwoo_cleaning_jobs_core.regions import RegionSet
from oomwoo_cleaning_jobs_core.segmentation import segment
from oomwoo_cleaning_jobs_core.validation import validate_region_set

class EditorController:
    def __init__(self, store=None):
        self.store = store or RegionSetStore()
        self.source = None
        self.constraints = ConstraintSet()
        self.regions = None

    def set_source(self, source):
        self.source = source
        loaded = self.store.load_draft(source)
        if loaded:
            self.regions, self.constraints = loaded.region_set, loaded.constraints
            return '已加载此地图的草稿'
        self.regions = None
        self.constraints = ConstraintSet()
        others = self.store.other_map_set_count(source)
        return f'当前地图没有区域集；磁盘上存在 {others} 份属于其他地图的区域集；请生成候选区域'

    def generate_candidates(self):
        self._require_source()
        keepout = self.constraints.mask_for(self.source)
        result = segment(self.source, cleanable_mask=self.source.free_mask() & ~keepout)
        self.regions = RegionSet.from_segmentation(result, self.source.resolution, self.source.origin,
            base_cleanable=self.source.free_mask(), keepout_mask=keepout)
        return result

    def paint_cell(self, label, row, col, erase=False):
        self._require_regions()
        stroke = np.zeros(self.source.cells.shape, dtype=bool)
        if 0 <= row < self.source.height and 0 <= col < self.source.width:
            stroke[row, col] = True
        if erase:
            return self.regions.erase(label, stroke), '已擦除'
        before = self.regions.labels.copy()
        changed = self.regions.paint(label, stroke)
        preempted = bool(changed and np.any((before != 0) & stroke & (before != label)))
        return changed, '已抢占其他 Region 的 cell' if preempted else ('已绘制' if changed else '笔画完全落在不可清扫空间')

    def create_rectangle(self, start_row, start_col, end_row, end_col, name):
        """以两个对角 cell 创建具名 Region；核心负责裁剪和抢占。"""
        self._require_source()
        if self.regions is None:
            self.generate_candidates()
        row0, row1 = sorted((int(start_row), int(end_row)))
        col0, col1 = sorted((int(start_col), int(end_col)))
        stroke = np.zeros(self.source.cells.shape, dtype=bool)
        stroke[row0:row1 + 1, col0:col1 + 1] = True
        label = self.regions.create(stroke, name)
        return label, ('已创建 Region' if label is not None
                       else '矩形完全落在不可清扫空间')

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
            raise ValueError(f'约束 {identifier!r} 不存在')
        self._set_constraints(ConstraintSet(keepouts, walls))

    def _set_constraints(self, constraints):
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
        if self.source is None: raise ValueError('请先打开地图')
    def _require_regions(self):
        self._require_source()
        if self.regions is None: raise ValueError('请先生成候选区域')
