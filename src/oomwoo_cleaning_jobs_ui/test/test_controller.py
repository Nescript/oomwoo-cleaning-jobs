import numpy as np
from oomwoo_cleaning_jobs_core.source_map import FREE, SourceMap
from oomwoo_cleaning_jobs_ui.controller import EditorController

def _source():
    return SourceMap(.1, 30, 30, (0.,0.,0.), np.full((30,30),FREE,dtype=np.int8))
def test_constraints_clip_regions_and_removal_does_not_restore_cells(tmp_path):
    controller=EditorController(); source=_source(); controller.store.root=tmp_path
    controller.set_source(source); controller.generate_candidates()
    label=controller.regions.regions()[0].label
    assert controller.regions.labels[15,15]==label
    controller.add_keepout('table',((1.4,1.4),(1.7,1.4),(1.7,1.7),(1.4,1.7)))
    assert controller.constraints.keepouts[0].identifier=='table'
    assert controller.regions.labels[15,15]==0
    controller.remove_constraint('table')
    assert controller.regions.cleanable[15,15]
    assert controller.regions.labels[15,15]==0

def test_set_source_reports_other_map_region_sets(tmp_path):
    first = EditorController(); first.store.root = tmp_path
    source_a = _source()
    first.set_source(source_a); first.generate_candidates(); first.save_draft()

    changed = source_a.cells.copy(); changed[0, 0] = 100
    source_b = SourceMap(source_a.resolution, source_a.width, source_a.height,
                         source_a.origin, changed)
    second = EditorController(); second.store.root = tmp_path

    message = second.set_source(source_b)

    assert '1 region set(s) on disk belong to other maps' in message


def test_named_rectangle_creates_full_stroke(tmp_path):
    controller = EditorController(); controller.store.root = tmp_path
    controller.set_source(_source()); controller.generate_candidates()

    label, message = controller.create_rectangle(4, 5, 7, 9, 'Dining Area')

    assert label is not None
    assert message == 'Region created'
    assert (controller.regions.labels[4:8, 5:10] == label).all()
    assert controller.regions.names[label] == 'Dining Area'


def test_virtual_wall_is_stored_and_applied(tmp_path):
    controller=EditorController(); controller.store.root=tmp_path; controller.set_source(_source()); controller.generate_candidates()
    controller.add_virtual_wall('wall',(1.0,1.5),(2.0,1.5),.1)
    assert controller.constraints.virtual_walls[0].identifier=='wall'
    assert controller.constraints.mask_for(controller.source).any()
