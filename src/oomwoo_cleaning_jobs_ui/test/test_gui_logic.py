"""GUI 逻辑验证测试（offscreen）：画布坐标映射、候选命名健壮性、编辑前置门控、拆分反馈。"""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import numpy as np
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtWidgets import QApplication, QInputDialog
from PyQt5.QtCore import QEvent

from oomwoo_cleaning_jobs_core.source_map import FREE, OCCUPIED, UNKNOWN, SourceMap
from oomwoo_cleaning_jobs_ui.app import Window

# 进程级单例：局部变量持有的 QApplication 被 GC 后 Qt 会销毁全部 widget
_APP = QApplication.instance() or QApplication([])


def _two_room_source() -> SourceMap:
    """30x60 cells @0.1m：内墙 col 29，门 rows 13-16。"""
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
    window.controller.set_source(source)
    window.refresh()
    return window


def _click(window: Window, row: int, col: int) -> None:
    """按当前画布映射关系模拟点击 cell (row, col) 的中心像素。"""
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
    """新建矩形区域不应被'尚无候选区域'门控挡住（controller 会自动分割）。"""
    window = _window_with(_two_room_source())
    assert window.controller.regions is None  # 尚未自动分割
    window._pending_name = '手画区'
    window.mode = 'create_rectangle'

    _click(window, 4, 40)
    _click(window, 8, 50)

    names = window.controller.regions.names if window.controller.regions else {}
    assert '手画区' in names.values(), '两点点击在分割前被画布门控吞掉了'
    window.close()


def test_name_candidates_survives_deleted_candidate(monkeypatch):
    """候选被删除/合并后，_unnamed_candidates 里的陈旧 label 不得引发 KeyError。"""
    window = _window_with(_two_room_source())
    window.generate_candidates.__wrapped__ if False else None
    window.controller.generate_candidates()
    window.refresh()
    labels = [r.label for r in window.controller.regions.regions()]
    assert len(labels) == 2
    window._unnamed_candidates = set(labels)
    # 用户在命名流程外删掉了一个候选
    window.controller.regions.delete(labels[0])

    monkeypatch.setattr(QInputDialog, 'getText', lambda *a, **k: ('客厅', True))
    window.name_candidates()  # 不应抛 KeyError

    assert window.controller.regions.names[labels[1]] == '客厅'
    window.close()


def test_split_gives_feedback_on_invalid_cut(monkeypatch):
    """无效切割（没把 Region 分成两片）必须给用户反馈。"""
    window = _window_with(_two_room_source())
    window.controller.generate_candidates()
    window.refresh()
    label = window.controller.regions.regions()[0].label
    window.select_label(label)
    monkeypatch.setattr(QInputDialog, 'getInt',
                        lambda *a, **k: (0, True))  # row 0：不穿任何 Region
    window.split()
    assert '未' in window.status.text() or '无效' in window.status.text(), \
        f'无效拆分没有反馈：{window.status.text()!r}'
    window.close()


def test_click_paint_erase_maps_to_correct_cell():
    """画布点击坐标映射：擦除模式下点击 cell (10,10) 应恰好清掉该 cell。"""
    window = _window_with(_two_room_source())
    window.controller.generate_candidates()
    window.refresh()
    label = window.controller.regions.labels[10, 10]
    assert label != 0
    window.select_label(int(label))
    window.set_mode('erase')

    _click(window, 10, 10)

    assert window.controller.regions.labels[10, 10] == 0, \
        '点击映射到了错误的 cell'
    window.close()
