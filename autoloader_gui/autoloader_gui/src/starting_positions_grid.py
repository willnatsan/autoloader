from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import (
    QBrush,
    QColor,
    QPen,
)
from PySide6.QtWidgets import (
    QGraphicsScene,
    QGraphicsView,
    QGraphicsRectItem,
)
from PySide6.QtWidgets import QApplication

from autoloader_data import *
from autoloader_interfaces.msg import Landmark

import logging

logger = logging.getLogger(__name__)


class CarrierGridBox(QGraphicsRectItem):
    def __init__(
        self,
        position: tuple[int, int],
        size: tuple[int, int],
        slot: list[int, int] | None = None,
        carrier: Landmark | None = None,
        parent=None,
    ) -> None:
        super().__init__()

        self.parent = parent

        self.x_pos = position[0]
        self.y_pos = position[1]
        self.width = size[0]
        self.height = size[1]
        self.slot = slot

        self.brush_curr = QBrush(Qt.white)
        self.pen_curr = QPen(Qt.black, 0)

        if self.slot in reshape(carrier.run_slots, 2):
            self.brush_curr.setColor(Qt.green)
        elif self.slot in reshape(carrier.failed_slots, 2):
            self.brush_curr.setColor(Qt.red)
        else:
            self.brush_curr.setColor(Qt.white)

        self.setRect(self.x_pos, self.y_pos, self.width, self.height)
        self.setBrush(self.brush_curr)
        self.setPen(self.pen_curr)


class CarrierGridView(QGraphicsView):
    device_count = 1

    def __init__(self, carrier: Landmark, parent=None) -> None:
        super().__init__()

        self.parent = parent
        self.carrier = carrier
        self.available_slots = reshape(carrier.available_slots, 2)
        self.run_slots = reshape(carrier.run_slots, 2)
        self.failed_slots = reshape(carrier.failed_slots, 2)
        self.select_failures = False

        CarrierGridView.device_count = len(self.run_slots)

        self.scene_width = self.width() * 0.75
        self.scene_height = self.height() * 0.75
        self.scene_view = QGraphicsScene(-50, 10, self.scene_width, self.scene_height)
        self.scene_view.setBackgroundBrush(QBrush(QColor(70, 70, 70)))

        self.num_cols, self.num_rows = self.available_slots[-1]
        self.device_size = min(
            self.scene_width // self.num_cols, self.scene_height // self.num_rows
        )

        self.setScene(self.scene_view)

        self.timer = QTimer()
        self.timer.timeout.connect(self.draw_carrier)
        self.timer.setInterval(250)
        self.timer.start()

    def box_toggle(self, slot: list[int, int]) -> None:
        if not self.select_failures:
            if slot in self.run_slots and len(self.run_slots) > 1:
                self.run_slots.remove(slot)
            elif slot not in self.run_slots and slot in self.available_slots:
                self.run_slots.append(slot)
                if slot in self.failed_slots:
                    self.failed_slots.remove(slot)
        elif self.select_failures:
            if slot in self.failed_slots:
                self.failed_slots.remove(slot)
            elif slot not in self.failed_slots and slot in self.available_slots:
                self.failed_slots.append(slot)
                if slot in self.run_slots:
                    self.run_slots.remove(slot)

        self.carrier.run_slots = flatten(self.run_slots)
        self.carrier.failed_slots = flatten(self.failed_slots)

    def switch_carrier(self, carrier: Landmark) -> None:
        self.carrier = carrier
        self.available_slots = reshape(carrier.available_slots, 2)
        self.run_slots = reshape(carrier.run_slots, 2)
        self.failed_slots = reshape(carrier.failed_slots, 2)

        self.num_cols, self.num_rows = self.available_slots[-1]
        self.device_size = min(
            self.scene_width // self.num_cols, self.scene_height // self.num_rows
        )

    def draw_carrier(self) -> None:
        self.scene_view.clear()
        self.update_carrier()

        x_increment = y_increment = self.device_size
        x_offset = y_offset = 0

        if (
            self.carrier.id == LandmarkID.SMD_CARRIER
            or self.carrier.id == LandmarkID.TO_CARRIER
        ):
            x_increment = -self.device_size
            x_offset = self.device_size * self.num_cols

        for slot in self.available_slots:
            box = CarrierGridBox(
                position=(
                    slot[0] * x_increment + x_offset,
                    slot[1] * y_increment + y_offset,
                ),
                size=(self.device_size, self.device_size),
                slot=slot,
                carrier=self.carrier,
                parent=self,
            )
            self.scene_view.addItem(box)

        self.scene_view.update()

    def mousePressEvent(self, event) -> None:
        clicked_pos = event.pos()
        box = self.itemAt(clicked_pos)

        self.update_carrier(box)

    def reset(self) -> None:
        self.run_slots = [[0, 0]]
        self.failed_slots = []
        CarrierGridView.device_count = 1

        self.carrier.run_slots = flatten(self.run_slots)
        self.carrier.failed_slots = flatten(self.failed_slots)

    def update_carrier(self, box: CarrierGridBox | None = None) -> None:
        pass


class OutputCarrierGridView(CarrierGridView):
    def __init__(self, carrier: Landmark, parent=None) -> None:
        super().__init__(carrier, parent)

    def switch_carrier(self, carrier: Landmark) -> None:
        super().switch_carrier(carrier)

    def update_carrier(self, box: CarrierGridBox | None = None) -> None:
        if self.select_failures and box is not None:
            if box.slot not in self.failed_slots:
                self.failed_slots.append(box.slot)
                if box.slot in self.run_slots:
                    self.run_slots.remove(box.slot)
                    CarrierGridView.device_count -= 1
            elif box.slot in self.failed_slots:
                self.failed_slots.remove(box.slot)
                self.compute_output_slots(box, ignore_box=True)
        else:
            self.compute_output_slots(box)

        self.carrier.failed_slots = flatten(self.failed_slots)
        self.carrier.run_slots = flatten(self.run_slots)

    def compute_output_slots(
        self, box: CarrierGridBox | None = None, ignore_box: bool = False
    ) -> None:
        start_x, start_y = self.run_slots[0]
        if box is not None:
            if box.slot in self.failed_slots:
                return
            if not ignore_box:
                start_x, start_y = box.slot

        run_slots_cp = self.run_slots.copy()
        count = 0

        self.run_slots.clear()

        if (
            self.carrier.id == LandmarkID.SMD_WAFFLE
            or self.carrier.id == LandmarkID.TO_WAFFLE
        ):
            for i in range(start_y, self.num_rows + 1):
                for j in range(0, self.num_cols + 1):
                    if count >= CarrierGridView.device_count:
                        break
                    if [j, i] not in self.available_slots:
                        continue
                    if [j, i] in self.failed_slots:
                        continue
                    if j < start_x and i == start_y:
                        continue

                    self.run_slots.append([j, i])
                    count += 1
                else:
                    continue
                break
        elif (
            self.carrier.id == LandmarkID.SMD_CARRIER
            or self.carrier.id == LandmarkID.TO_CARRIER
        ):
            for j in range(start_x, self.num_cols + 1):
                for i in range(0, self.num_rows + 1):
                    if count >= CarrierGridView.device_count:
                        break
                    if [j, i] not in self.available_slots:
                        continue
                    if [j, i] in self.failed_slots:
                        continue
                    if i < start_y and j == start_x:
                        continue

                    self.run_slots.append([j, i])
                    count += 1
                else:
                    continue
                break

        if count < CarrierGridView.device_count:
            self.run_slots = run_slots_cp

        if (
            self.carrier.id == LandmarkID.SMD_WAFFLE
            or self.carrier.id == LandmarkID.TO_WAFFLE
        ):
            self.run_slots = sorted(self.run_slots, key=lambda slot: (slot[1], slot[0]))
        elif (
            self.carrier.id == LandmarkID.SMD_CARRIER
            or self.carrier.id == LandmarkID.TO_CARRIER
        ):
            self.run_slots = sorted(self.run_slots, key=lambda slot: (slot[0], slot[1]))

        self.carrier.run_slots = flatten(self.run_slots)


class InputCarrierGridView(CarrierGridView):
    def __init__(
        self, carrier: Landmark, output_carrier: Landmark, parent=None
    ) -> None:
        super().__init__(carrier, parent)

        self.output_carrier = output_carrier
        self.max_device_count = min(
            len(reshape(self.carrier.available_slots, 2)),
            len(reshape(self.output_carrier.available_slots, 2)),
        )

    def switch_carrier(self, carrier: Landmark, output_carrier: Landmark) -> None:
        super().switch_carrier(carrier)

        self.output_carrier = output_carrier
        self.max_device_count = min(
            len(reshape(self.carrier.available_slots, 2)),
            len(reshape(self.output_carrier.available_slots, 2)),
        )

    def update_carrier(self, box: CarrierGridBox | None = None) -> None:
        while len(self.run_slots) > CarrierGridView.device_count:
            self.run_slots.pop()

        if box is not None:
            modifier = QApplication.keyboardModifiers()
            if modifier == Qt.ShiftModifier:
                if not self.select_failures:
                    start_x, start_y = self.run_slots[0]
                    if box.slot not in self.run_slots:
                        start_x, start_y = self.run_slots[-1]
                elif self.select_failures and self.failed_slots:
                    start_x, start_y = self.failed_slots[0]
                    if box.slot not in self.failed_slots:
                        start_x, start_y = self.failed_slots[-1]
                elif self.select_failures and not self.failed_slots:
                    start_x, start_y = self.run_slots[-1]
                else:
                    start_x, start_y = [0, 0]

                if (
                    self.carrier.id == LandmarkID.SMD_WAFFLE
                    or self.carrier.id == LandmarkID.TO_WAFFLE
                ):
                    for i in range(start_y, box.slot[1] + 1):
                        for j in range(0, self.num_cols + 1):
                            if [j, i] == [
                                start_x,
                                start_y,
                            ] and [j, i] not in self.failed_slots:
                                continue
                            if j < start_x and i == start_y:
                                continue
                            if j < box.slot[0] or i < box.slot[1]:
                                self.box_toggle([j, i])
                elif (
                    self.carrier.id == LandmarkID.SMD_CARRIER
                    or self.carrier.id == LandmarkID.TO_CARRIER
                ):
                    for j in range(start_x, box.slot[0] + 1):
                        for i in range(0, self.num_rows + 1):
                            if [j, i] == [
                                start_x,
                                start_y,
                            ] and [j, i] not in self.failed_slots:
                                continue
                            if i < start_y and j == start_x:
                                continue
                            if i < box.slot[1] or j < box.slot[0]:
                                self.box_toggle([j, i])

            self.box_toggle(box.slot)

            while len(self.run_slots) > self.max_device_count:
                self.run_slots.pop()

            CarrierGridView.device_count = len(self.run_slots)

        if (
            self.carrier.id == LandmarkID.SMD_WAFFLE
            or self.carrier.id == LandmarkID.TO_WAFFLE
        ):
            self.run_slots = sorted(self.run_slots, key=lambda slot: (slot[1], slot[0]))
        elif (
            self.carrier.id == LandmarkID.SMD_CARRIER
            or self.carrier.id == LandmarkID.TO_CARRIER
        ):
            self.run_slots = sorted(self.run_slots, key=lambda slot: (slot[0], slot[1]))

        self.carrier.run_slots = flatten(self.run_slots)
