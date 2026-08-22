"""Standalone PyQt5 editor application."""
from __future__ import annotations
import sys, threading
import numpy as np
from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QFileDialog, QGroupBox, QHBoxLayout, QInputDialog, QLabel, QListWidget, QListWidgetItem, QMainWindow, QMessageBox, QPushButton, QVBoxLayout, QWidget
from oomwoo_cleaning_jobs_core.map_io import load_map_file
from .controller import EditorController
from .ros_map_source import RosMapSource

PALETTE=[(55,126,184),(228,26,28),(77,175,74),(152,78,163),(255,127,0)]
class LiveMapBridge(QObject):
    received=pyqtSignal(object)

class MapCanvas(QLabel):
    def __init__(self, window): super().__init__(); self.window=window; self.setMinimumSize(600,500); self.setAlignment(Qt.AlignCenter)
    def mousePressEvent(self,event):
        c=self.window.controller; pix=self.pixmap()
        if not c.source or not pix:return
        x=(event.pos().x()-(self.width()-pix.width())//2)*c.source.width//pix.width(); y=(event.pos().y()-(self.height()-pix.height())//2)*c.source.height//pix.height(); row,col=c.source.height-1-y,x
        if not(0<=row<c.source.height and 0<=col<c.source.width):return
        try:
            if self.window.mode in ('create_rectangle', 'virtual_wall'):
                self.window.handle_two_point_action(row, col)
                return
            # 两点动作只需要地图；画笔/擦除才要求已生成候选并选中 Region
            if not c.regions or self.window.selected_label() is None:return
            _,msg=c.paint_cell(self.window.selected_label(),row,col,self.window.mode=='erase')
            self.window.status.setText(msg); self.window.refresh()
        except ValueError as exc:self.window.error(str(exc))

class Window(QMainWindow):
    def __init__(self):
        super().__init__(); self.controller=EditorController(); self.mode='paint'; self._first_point=None; self._pending_name=None; self._pending_wall_width=None; self._unnamed_candidates=set(); self._refreshing=False; self._executor=self._node=self._thread=None
        self.bridge=LiveMapBridge(); self.bridge.received.connect(self.receive_live_map)
        self.setWindowTitle('OOMWOO Region Set Editor'); root=QWidget(); self.setCentralWidget(root); layout=QHBoxLayout(root); left=QVBoxLayout(); layout.addLayout(left); self.canvas=MapCanvas(self); layout.addWidget(self.canvas,1)
        for title,fn in [('打开地图文件',self.open_file),('启动 /map',self.toggle_live),('1. 自动分割',self.generate),('2. 逐个命名候选',self.name_candidates),('保存草稿',self.save),('校验/发布',self.publish)]:
            b=QPushButton(title); b.clicked.connect(fn); left.addWidget(b)
        self.list=QListWidget(); left.addWidget(self.list); self.list.itemSelectionChanged.connect(self.on_selection_changed)
        self.advanced_toggle=QPushButton('显示高级编辑'); self.advanced_toggle.clicked.connect(self.toggle_advanced); left.addWidget(self.advanced_toggle)
        self.advanced=QGroupBox('高级编辑（仅在候选有误时使用）'); advanced_layout=QVBoxLayout(self.advanced)
        for title,fn in [('新建区域',self.begin_rectangle),('绘制',lambda:self.set_mode('paint')),('擦除',lambda:self.set_mode('erase')),('重命名当前 Region',self.rename),('删除 Region',self.delete),('合并到…',self.merge),('拆分(横切)',self.split),('添加 Keepout',self.add_keepout),('添加 Virtual Wall',self.begin_wall),('删除约束',self.remove_constraint)]:
            b=QPushButton(title); b.clicked.connect(fn); advanced_layout.addWidget(b)
        self.advanced.setVisible(False); left.addWidget(self.advanced)
        self.status=QLabel('打开 map.yaml 或启动 /map'); left.addWidget(self.status); self.resize(1050,700)
    def selected_label(self):
        item=self.list.currentItem(); return int(item.data(Qt.UserRole)) if item else None
    def on_selection_changed(self):
        if not self._refreshing:
            self.refresh()
    def set_mode(self,mode):
        self.mode=mode; self._first_point=None; self.status.setText('模式：'+mode)
    def begin_rectangle(self):
        name,ok=QInputDialog.getText(self,'新建区域','区域名称：')
        if ok and name.strip():
            self.mode='create_rectangle'; self._pending_name=name.strip(); self._first_point=None
            self.status.setText('点击矩形左上角，再点击右下角')
    def begin_wall(self):
        name,ok=QInputDialog.getText(self,'Virtual Wall','名称：')
        if not (ok and name.strip()): return
        width,ok=QInputDialog.getDouble(self,'Virtual Wall','宽度（m）：',0.05,0.001)
        if ok:
            self.mode='virtual_wall'; self._pending_name=name.strip(); self._pending_wall_width=width; self._first_point=None
            self.status.setText('点击虚拟墙起点，再点击终点')
    def handle_two_point_action(self,row,col):
        if self._first_point is None:
            self._first_point=(row,col); self.status.setText('已记录第一个点，请点击第二个点'); return
        first=self._first_point; self._first_point=None
        if self.mode=='create_rectangle':
            _label,msg=self.controller.create_rectangle(*first,row,col,self._pending_name)
            self.status.setText(msg); self.mode='paint'; self.refresh(); return
        start=self.map_point(*first); end=self.map_point(row,col)
        self.controller.add_virtual_wall(self._pending_name,start,end,self._pending_wall_width)
        self.status.setText('Virtual Wall 已添加；Region 已即时裁剪'); self.mode='paint'; self.refresh()
    def map_point(self,row,col):
        import math
        source=self.controller.source; local_x=(col+.5)*source.resolution; local_y=(row+.5)*source.resolution; ox,oy,yaw=source.origin
        return (ox+math.cos(yaw)*local_x-math.sin(yaw)*local_y, oy+math.sin(yaw)*local_x+math.cos(yaw)*local_y)
    def open_file(self):
        path,_=QFileDialog.getOpenFileName(self,'打开 nav2 map.yaml','','YAML (*.yaml *.yml)')
        if path:
            try:self.replace_source(load_map_file(path))
            except Exception as exc:self.error(str(exc))
    def toggle_live(self): self.stop_live() if self._node else self.start_live()
    def start_live(self):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            if not rclpy.ok():rclpy.init()
            self._node=RosMapSource(self.bridge.received.emit);self._executor=SingleThreadedExecutor();self._executor.add_node(self._node)
            self._thread=threading.Thread(target=self._executor.spin,daemon=True);self._thread.start();self.status.setText('正在订阅 transient-local /map')
        except Exception as exc:self.stop_live();self.error(str(exc))
    def stop_live(self):
        if self._executor:self._executor.shutdown()
        if self._node:self._node.destroy_node()
        if self._thread:self._thread.join(timeout=2)
        self._executor=self._node=self._thread=None
        try:
            import rclpy
            if rclpy.ok():rclpy.shutdown()
        except Exception:pass
        self.status.setText('/map 已停止')
    def receive_live_map(self,source):
        self.replace_source(source)

    def replace_source(self, source):
        """Preserve an unchanged session; confirm before replacing a different map."""
        current = self.controller.source
        if current is not None and current.identity == source.identity:
            self.status.setText('地图 identity 未变；保留当前编辑会话')
            return False
        if current is not None:
            answer = QMessageBox.question(
                self, '地图已变化',
                '地图 identity 已改变；替换当前编辑会切换地图且不迁移区域。是否继续？',
                QMessageBox.Yes | QMessageBox.No)
            if answer != QMessageBox.Yes:
                return False
        self.status.setText(self.controller.set_source(source))
        self.refresh()
        return True
    def generate(self):
        try:
            self.controller.generate_candidates(); self.mode='paint'
            self._unnamed_candidates={info.label for info in self.controller.regions.regions()}
            self.refresh(); self.name_candidates()
        except Exception as exc:self.error(str(exc))
    def toggle_advanced(self):
        visible=not self.advanced.isVisible(); self.advanced.setVisible(visible)
        self.advanced_toggle.setText('隐藏高级编辑' if visible else '显示高级编辑')
    def select_label(self, label):
        for index in range(self.list.count()):
            if self.list.item(index).data(Qt.UserRole)==label:
                self.list.setCurrentRow(index); return
    def name_candidates(self):
        if self.controller.regions is None:
            self.error('请先完成自动分割'); return
        # 丢弃命名流程外被删除/合并的陈旧候选 label，避免访问不存在的名称
        self._unnamed_candidates &= set(self.controller.regions.names)
        total=len(self._unnamed_candidates)
        while self._unnamed_candidates:
            label=min(self._unnamed_candidates); self.select_label(label)
            current=self.controller.regions.names[label]
            name,ok=QInputDialog.getText(self,'命名候选区域',f'候选 Region {label} 的名称：',text=current)
            if not ok:
                done=total-len(self._unnamed_candidates)
                self.status.setText(f'已命名 {done}/{total} 个候选；可稍后继续逐个命名')
                return
            if not name.strip():
                self.error('区域名称不能为空'); return
            self.controller.regions.rename(label,name.strip()); self._unnamed_candidates.remove(label); self.refresh()
        self.status.setText(f'已完成 {total} 个候选的命名；可保存草稿或校验/发布')
    def _coordinates(self,prompt):
        text,ok=QInputDialog.getMultiLineText(self,'坐标',prompt)
        if not ok:return None
        return [tuple(map(float,line.replace(',',' ').split())) for line in text.splitlines() if line.strip()]
    def add_keepout(self):
        try:
            ident,ok=QInputDialog.getText(self,'Keepout','名称：'); points=self._coordinates('每行一个 map-frame x,y（至少三行）：') if ok else None
            if ident and points:self.controller.add_keepout(ident,points);self.status.setText('Keepout 已添加；Region 已即时裁剪');self.refresh()
        except Exception as exc:self.error(str(exc))
    def remove_constraint(self):
        names=[x.identifier for x in self.controller.constraints.keepouts]+[x.identifier for x in self.controller.constraints.virtual_walls]
        if names:
            ident,ok=QInputDialog.getItem(self,'删除约束','约束：',names,0,False)
            if ok:
                try:self.controller.remove_constraint(ident);self.status.setText('约束已移除；被裁掉的 Region cell 不会复活');self.refresh()
                except Exception as exc:self.error(str(exc))
    def save(self):
        try:self.controller.save_draft();self.status.setText('草稿已保存')
        except Exception as exc:self.error(str(exc))
    def publish(self):
        try:
            report=self.controller.report();text='\n'.join(f'[{i.level}] {i.message}' for i in report.issues) or '无问题'
            if report.errors:self.error(text);return
            self.controller.publish();QMessageBox.information(self,'已发布',text)
        except Exception as exc:self.error(str(exc))
    def rename(self):
        label=self.selected_label()
        if label:
            name,ok=QInputDialog.getText(self,'重命名','名称：',text=self.controller.regions.names[label])
            if ok and name:
                self.controller.regions.rename(label,name); self._unnamed_candidates.discard(label); self.refresh()
    def delete(self):
        if self.selected_label():
            self._unnamed_candidates.discard(self.selected_label())
            self.controller.regions.delete(self.selected_label());self.refresh()
    def merge(self):
        source=self.selected_label()
        if source:
            target,ok=QInputDialog.getInt(self,'合并','目标 label：',1)
            if ok:
                try:
                    self.controller.regions.merge(target,source)
                    self._unnamed_candidates.discard(source)
                    self.refresh()
                except ValueError as exc:self.error(str(exc))
    def split(self):
        label=self.selected_label()
        if label:
            row,ok=QInputDialog.getInt(self,'拆分','切割 row：',self.controller.source.height//2,0,self.controller.source.height-1)
            if ok:
                cut=np.zeros(self.controller.source.cells.shape,dtype=bool);cut[row,:]=True
                pieces=self.controller.regions.split(label,cut)
                self.status.setText(f'已拆分为 {len(pieces)} 片' if pieces else '切割线未把 Region 分成两片')
                self.refresh()
    def refresh(self):
        c=self.controller; selected=self.selected_label(); self._refreshing=True; self.list.clear()
        if not c.source:
            self._refreshing=False; self.canvas.clear(); return
        if c.regions:
            for info in c.regions.regions():
                item=QListWidgetItem(f'{info.label}: {info.name} ({info.area_m2:.2f} m²)'); item.setData(Qt.UserRole,info.label); self.list.addItem(item)
            if selected is not None: self.select_label(selected)
        self._refreshing=False
        cells=c.source.cells;image=np.zeros((c.source.height,c.source.width,3),dtype=np.uint8);image[cells<0]=(70,70,70);image[cells>=25]=(20,20,20);image[(cells>=0)&(cells<25)]=(235,235,235)
        mask=c.constraints.mask_for(c.source);image[mask]=(210,30,180)
        if c.regions:
            for info in c.regions.regions():image[c.regions.mask_of(info.label)]=PALETTE[(info.label-1)%len(PALETTE)]
            image[c.regions.unassigned_cleanable_mask]=(245,220,100);image[mask]=(210,30,180)
        # PyQt5 不接受 NumPy 的 memoryview；传入 bytes 后立即 copy，避免
        # QImage 持有临时 NumPy 缓冲区。
        pixels = np.ascontiguousarray(image[::-1]).tobytes()
        q = QImage(pixels, c.source.width, c.source.height,
                   c.source.width * 3, QImage.Format_RGB888).copy()
        self.canvas.setPixmap(QPixmap.fromImage(q).scaled(
            self.canvas.size(), Qt.KeepAspectRatio, Qt.FastTransformation))
    def closeEvent(self,event):self.stop_live();event.accept()
    def error(self,text):QMessageBox.critical(self,'OOMWOO Region Set Editor',text)
def main(argv=None):
    app=QApplication(argv or sys.argv);window=Window();window.show();return app.exec_()
