"""Qt canvas rendering regression tests."""

import os
import threading
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
from PyQt5.QtWidgets import QApplication, QInputDialog

from fake_segmentation import fake_segmentation
from oomwoo_segmentation.source_map import FREE, SourceMap
from oomwoo_cleaning_jobs_ui.app import Window


def test_refresh_renders_source_map_without_qimage_buffer_error():
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.controller.segmenter = fake_segmentation
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


def test_auto_segmentation_runs_off_the_qt_thread(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window()
    source = SourceMap(
        resolution=0.1,
        width=10,
        height=10,
        origin=(0.0, 0.0, 0.0),
        cells=np.full((10, 10), FREE, dtype=np.int8),
    )
    called_on = []

    def recording_segmenter(*args, **kwargs):
        called_on.append(threading.get_ident())
        return fake_segmentation(*args, **kwargs)

    window.controller.segmenter = recording_segmenter
    window.controller.set_source(source)
    monkeypatch.setattr(window, 'name_candidates', lambda: None)
    main_thread = threading.get_ident()

    window.generate()
    deadline = time.monotonic() + 2.0
    while window._segmentation_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)

    assert called_on and called_on[0] != main_thread
    assert window.controller.regions is not None
    assert window._segmentation_thread is None
    window.close()
    app.processEvents()


def test_primary_flow_names_each_candidate_and_keeps_editing_advanced(monkeypatch):
    app = QApplication.instance() or QApplication([])
    window = Window()
    window.controller.segmenter = fake_segmentation
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
