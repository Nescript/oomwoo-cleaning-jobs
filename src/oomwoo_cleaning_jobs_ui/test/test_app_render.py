"""Qt canvas rendering regression tests."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
from PyQt5.QtWidgets import QApplication, QInputDialog

from oomwoo_cleaning_jobs_core.source_map import FREE, SourceMap
from oomwoo_cleaning_jobs_ui.app import Window


def test_refresh_renders_source_map_without_qimage_buffer_error():
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.controller.set_source(SourceMap(
        resolution=0.1,
        width=4,
        height=3,
        origin=(0.0, 0.0, 0.0),
        cells=np.full((3, 4), FREE, dtype=np.int8),
    ))

    window.refresh()

    assert window.canvas.pixmap() is not None
    window.close()
    app.processEvents()


def test_primary_flow_names_each_candidate_and_keeps_editing_advanced(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.controller.set_source(SourceMap(
        resolution=0.1,
        width=10,
        height=10,
        origin=(0.0, 0.0, 0.0),
        cells=np.full((10, 10), FREE, dtype=np.int8),
    ))
    window.controller.generate_candidates()
    label = window.controller.regions.regions()[0].label
    window._unnamed_candidates = {label}
    monkeypatch.setattr(QInputDialog, 'getText', lambda *args, **kwargs: ('Living Room', True))

    window.refresh()
    window.name_candidates()

    assert window.controller.regions.names[label] == 'Living Room'
    assert not window._unnamed_candidates
    assert window.advanced.isHidden()
    window.toggle_advanced()
    assert not window.advanced.isHidden()
    window.close()
    app.processEvents()
