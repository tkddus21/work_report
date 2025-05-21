from PyQt5.QtWidgets import QGraphicsView, QGraphicsRectItem
from PyQt5.QtCore import Qt, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import QPen, QColor
import json

class BoxDrawingView(QGraphicsView):
    boxUpdated = pyqtSignal()  # 박스가 추가되거나 삭제된 경우
    boxCreated = pyqtSignal(float)  # x좌표를 전달
    boxDeleted = pyqtSignal(dict) # 삭제될 박스의 절대 좌표 정보 전달

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)

        self.box_items = []
        self.selected_box = None

        self.drawing = False
        self.start_pos = QPointF()
        self.temp_box = None

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            scene_pos = self.mapToScene(event.pos())

            # 기존 박스를 클릭했는지 확인
            clicked_items = self.scene().items(scene_pos)
            for item in clicked_items:
                if isinstance(item, QGraphicsRectItem):
                    self.select_box(item)
                    return

            self.clear_selection()

            # 새 박스 그리기 시작
            self.drawing = True
            self.start_pos = scene_pos
            self.temp_box = QGraphicsRectItem(QRectF(self.start_pos, self.start_pos))
            self.temp_box.setPen(QPen(QColor(255, 0, 0), 2))  # 실선 빨간색
            self.temp_box.setZValue(1)  # 다른 아이템 위에 표시
            self.scene().addItem(self.temp_box)

        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.drawing and self.temp_box:
            end_pos = self.mapToScene(event.pos())
            rect = QRectF(self.start_pos, end_pos).normalized()
            self.temp_box.setRect(rect)
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.drawing and self.temp_box:
            rect = self.temp_box.rect()
            if rect.width() > 5 and rect.height() > 5:
                self.box_items.append(self.temp_box)
                
                # 박스 생성 시 x좌표 전달
                self.boxCreated.emit(rect.x()) 
                # self.boxUpdated.emit() # main에서 저장하므로 필요없음
            else:
                self.scene().removeItem(self.temp_box)  # 너무 작으면 제거

            self.temp_box = None
            self.drawing = False
        else:
            super().mouseReleaseEvent(event)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete and self.selected_box:
            # 삭제할 박스의 절대 좌표 정보 가져오기
            box_data_to_delete = self.selected_box.data(1) # data(1)에 절대 좌표 저장

            # 박스 제거
            self.scene().removeItem(self.selected_box)
            if self.selected_box in self.box_items:
                self.box_items.remove(self.selected_box)
            
            self.selected_box = None
            
            # 삭제 신호 발생 (main에서 파일 업데이트 처리)
            if box_data_to_delete:
                 self.boxDeleted.emit(box_data_to_delete)
        else:
            super().keyPressEvent(event)

    def select_box(self, box_item):
        if self.selected_box:
            self.selected_box.setPen(QPen(QColor(255, 0, 0), 2))  # 선택 해제 → 빨간색

        self.selected_box = box_item
        box_item.setPen(QPen(QColor(0, 255, 0), 2))  # 선택 시 → 초록색

    def clear_selection(self):
        if self.selected_box:
            self.selected_box.setPen(QPen(QColor(255, 0, 0), 2))  # 원래 색 복구
            self.selected_box = None

    def create_box_item(self, x, y, width, height, absolute_data=None):
        """주어진 좌표와 크기로 QGraphicsRectItem 생성 및 추가"""
        box = QGraphicsRectItem(x, y, width, height)
        box.setPen(QPen(QColor(255, 0, 0), 2))  # 빨간색 실선
        box.setZValue(1)  # 다른 아이템 위에 표시
        if absolute_data:
            box.setData(1, absolute_data) # data(1)에 절대 좌표 정보 저장
        self.scene().addItem(box)
        self.box_items.append(box)
        return box

    def load_boxes_absolute(self, filepath, current_buffer_offset, viewer_width):
        """파일에서 모든 박스를 불러와 현재 버퍼 범위에 해당하는 박스만 그리기"""
        self.clear_all_boxes() # 기존 박스 제거

        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            all_boxes = data.get("boxes", [])
            print(f"[📂 전체 박스 불러오기] {len(all_boxes)}개 박스")

            for box_data in all_boxes:
                x_absolute = box_data["x_absolute"]
                y = box_data["y"]
                width = box_data["width"]
                height = box_data["height"]

                # 현재 버퍼 범위에 포함되는지 확인
                # 뷰어의 절대 시작 위치: self.window().ui.horizontalScrollBar.value()
                # 뷰어의 절대 끝 위치: self.window().ui.horizontalScrollBar.value() + viewer_width
                viewer_abs_start = self.window().ui.horizontalScrollBar.value()
                viewer_abs_end = viewer_abs_start + viewer_width
                
                # 박스의 절대 범위: x_absolute 부터 x_absolute + width
                box_abs_start = x_absolute
                box_abs_end = x_absolute + width

                # 박스가 뷰어 범위와 겹치는지 확인
                if not (box_abs_end < viewer_abs_start or box_abs_start > viewer_abs_end):
                    # 현재 버퍼 기준 상대 좌표 계산: 박스 절대 위치 - 현재 버퍼 시작 절대 오프셋
                    # x_local = x_absolute - current_buffer_offset # 이 계산은 틀림

                    # 현재 화면(뷰어) 기준 상대 좌표 계산: 박스 절대 위치 - 현재 스크롤바 값
                    x_local = x_absolute - self.window().ui.horizontalScrollBar.value()

                    # 박스 그리기
                    self.create_box_item(x_local, y, width, height, absolute_data=box_data) # 박스 생성 시 절대 좌표 정보도 함께 저장
                    # print(f"[✨ 박스 그림] 절대 X: {x_absolute}, 상대 X: {x_local}")

        except FileNotFoundError:
            print(f"[ℹ️ 파일 없음] {filepath} 파일 없음")
            pass # 파일이 없으면 박스 없음으로 처리
        except Exception as e:
            print(f"[❌ 박스 로드 실패] {e}")

    def save_boxes_absolute(self, filepath, box_data_list):
        """주어진 박스 데이터 리스트를 파일에 저장 (덮어쓰기)"""
        data = {"boxes": box_data_list}
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            # print(f"[💾 박스 전체 저장 완료] {filepath} ({len(box_data_list)}개)")
        except Exception as e:
            print(f"[❌ 박스 저장 실패] {e}")

    def add_box_absolute(self, filepath, box_data):
        """파일에 새로운 박스 데이터를 추가"""
        all_boxes = []
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                all_boxes = data.get("boxes", [])
        except (FileNotFoundError, json.JSONDecodeError):
            pass # 파일이 없거나 형식이 잘못되었으면 빈 리스트로 시작

        all_boxes.append(box_data)

        with open(filepath, 'w') as f:
            json.dump({"boxes": all_boxes}, f, indent=2)
        print(f"[➕ 박스 추가 완료] {filepath}")

    def delete_box_absolute(self, filepath, box_data_to_delete):
        """파일에서 특정 박스 데이터를 삭제"""
        all_boxes = []
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
                all_boxes = data.get("boxes", [])
        except (FileNotFoundError, json.JSONDecodeError):
            return # 파일이 없으면 삭제할 박스도 없음

        # 삭제할 박스와 일치하는 항목 찾기 (절대 좌표 기준)
        updated_boxes = [b for b in all_boxes if not (
            b.get("x_absolute") == box_data_to_delete.get("x_absolute") and
            b.get("y") == box_data_to_delete.get("y") and
            b.get("width") == box_data_to_delete.get("width") and
            b.get("height") == box_data_to_delete.get("height")
        )]

        if len(updated_boxes) < len(all_boxes):
             with open(filepath, 'w') as f:
                json.dump({"boxes": updated_boxes}, f, indent=2)
             print(f"[🗑️ 박스 삭제 완료] {filepath}")
        # else:
             # print("[ℹ️ 삭제할 박스 찾지 못함]")

    def clear_all_boxes(self):
        """모든 박스 제거"""
        for box in self.box_items:
            self.scene().removeItem(box)
        self.box_items.clear()
        self.selected_box = None

    # def draw_box_absolute(self, x, y, w, h, segment_id = None):
    #     """절대 좌표로 박스 그리기 - 이 함수는 이제 load_boxes_absolute 내부에서 사용"""
    #     rect = QRectF(x, y, w, h)
    #     box = QGraphicsRectItem(rect)
    #     box.setPen(QPen(QColor(255, 0, 0), 2))  # 빨간색 실선
    #     box.setZValue(1)  # 다른 아이템 위에 표시
    #     # if segment_id is not None:
    #     #     box.setData(0, segment_id)

    #     self.scene().addItem(box)
    #     self.box_items.append(box)
    #     return box