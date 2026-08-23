"""GUI logic tests (offscreen): canvas coordinate mapping, candidate naming robustness, edit preconditions, split feedback."""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtWidgets import QApplication, QInputDialog
from PyQt5.QtCore import QEvent

from fake_segmentation import fake_segmentation
from oomwoo_segmentation.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_cleaning_jobs_ui.app import Window

# Process-level singleton: a QApplication held by a local variable gets
# garbage-collected and Qt then destroys all widgets.
_APP = QApplication.instance() or QApplication([])


def _two_room_source() -> SourceMap:
    """30x60 cells @0.1m: interior wall at col 29, doorway rows 13-16."""
    cells = np.full((30, 60), UNKNOWN, dtype=np.int8)
    cells[2:28, 2:58] = FREE
    cells[1, 1:59] = OCCUPIED
    cells[28, 1:59] = OCCUPIED
    cells[1:29, 1] = OCCUPIED
    cells[1:29, 58] = OCCUPIED
    cells[2:28, 29] = OCCUPIED
    cells[13:17, 29] = FREE
    return SourceMap(0.1, 60, 30, (0.0, 0.0, 0.0), cells)


def _window_with(source: SourceMap) -> Window:
    window = Window()
    window.controller.segmenter = fake_segmentation
    window.controller.set_source(source)
    window.refresh()
    return window


def _click(window: Window, row: int, col: int) -> None:
    """Simulate a click at the center pixel of cell (row, col) under the current canvas mapping."""
    canvas = window.canvas
    pix = canvas.pixmap()
    source = window.controller.source
    offset_x = (canvas.width() - pix.width()) // 2
    offset_y = (canvas.height() - pix.height()) // 2
    y = source.height - 1 - row
    px = offset_x + int((col + 0.5) * pix.width() / source.width)
    py = offset_y + int((y + 0.5) * pix.height() / source.height)
    event = QMouseEvent(QEvent.MouseButtonPress, QPointF(px, py),
                        Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    canvas.mousePressEvent(event)


def test_canvas_two_point_action_works_before_segmentation():
    """Rectangle creation must not be blocked by the 'no candidates yet' gate (the controller auto-segments)."""
    window = _window_with(_two_room_source())
    assert window.controller.regions is None  # no auto segmentation yet
    window._pending_name = 'Hand-Drawn'
    window.mode = 'create_rectangle'

    _click(window, 4, 40)
    _click(window, 8, 50)

    names = window.controller.regions.names if window.controller.regions else {}
    assert 'Hand-Drawn' in names.values(), 'two-point click before segmentation was swallowed by the canvas gate'
    window.close()


def test_name_candidates_survives_deleted_candidate(monkeypatch):
    """After a candidate is deleted/merged, stale labels in _unnamed_candidates must not raise KeyError."""
    window = _window_with(_two_room_source())
    window.generate_candidates.__wrapped__ if False else None
    window.controller.generate_candidates()
    window.refresh()
    labels = [r.label for r in window.controller.regions.regions()]
    assert len(labels) == 2
    window._unnamed_candidates = set(labels)
    # The user deleted one candidate outside the naming flow.
    window.controller.regions.delete(labels[0])

    monkeypatch.setattr(QInputDialog, 'getText', lambda *a, **k: ('Living Room', True))
    window.name_candidates()  # must not raise KeyError

    assert window.controller.regions.names[labels[1]] == 'Living Room'
    window.close()


def test_split_gives_feedback_on_invalid_cut(monkeypatch):
    """An invalid cut (one that does not split the Region into two pieces) must give the user feedback."""
    window = _window_with(_two_room_source())
    window.controller.generate_candidates()
    window.refresh()
    label = window.controller.regions.regions()[0].label
    window.select_label(label)
    monkeypatch.setattr(QInputDialog, 'getInt',
                        lambda *a, **k: (0, True))  # row 0: crosses no Region
    window.split()
    assert 'did not split' in window.status.text(), \
        f'invalid split gave no feedback: {window.status.text()!r}'
    window.close()


def test_click_paint_erase_maps_to_correct_cell():
    """Canvas click coordinate mapping: clicking cell (10,10) in erase mode clears exactly that cell."""
    window = _window_with(_two_room_source())
    window.controller.generate_candidates()
    window.refresh()
    label = window.controller.regions.labels[10, 10]
    assert label != 0
    window.select_label(int(label))
    window.set_mode('erase')

    _click(window, 10, 10)

    assert window.controller.regions.labels[10, 10] == 0, \
        'click mapped to the wrong cell'
    window.close()
