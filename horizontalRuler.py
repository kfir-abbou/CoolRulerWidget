import sys
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsRectItem, QVBoxLayout, QWidget, QSlider
from PyQt5.QtCore import QRectF, Qt, QPropertyAnimation, QEasingCurve, pyqtProperty
from PyQt5.QtGui import QPen, QBrush, QColor
from PyQt5.QtCore import QObject  # Import QObject
from PyQt5.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QVBoxLayout, QWidget, QSlider
from PyQt5.QtCore import QRectF, Qt, QPropertyAnimation, pyqtProperty
from PyQt5.QtGui import QPen, QBrush, QColor


class AnimationHandler(QObject):  # Use QObject
    def __init__(self, target_item):
        super().__init__()
        self.target_item = target_item

    @pyqtProperty(QRectF)
    def rect(self):
        return self.target_item.rect()

    @rect.setter
    def rect(self, value):
        self.target_item.setRect(value)


class RulerSegment(QGraphicsRectItem):
    def __init__(self, x, y, width, height, color, val, parent=None):
        super().__init__(x, y, width, height, parent)
        # self.setAcceptHoverEvents(True)
        self.setBrush(QBrush(color))
        self.original_height = height
        self.parent = parent
        self.animation_handler = AnimationHandler(self)
        self.animation = QPropertyAnimation(self.animation_handler, b'rect')
        self.animation.setDuration(300)
        self.val = val

    def hoverEnterEvent(self, event):
        self.animate_height(60)  # Increase height on hover

    def hoverLeaveEvent(self, event):
        # Restore original height on leave
        self.animate_height(15)

    def animate_height(self, target_height):
        try:
            start_rect = self.rect()
            end_rect = QRectF(start_rect.x(), start_rect.y(
            ) - (target_height - start_rect.height()), start_rect.width(), target_height)
            self.animation.setStartValue(start_rect)
            self.animation.setEndValue(end_rect)
            self.animation.start()
        except Exception as e:
            print(e)

    def getHeight(self):
        return self.rect().height()

    def setHeight(self, height):
        self.setRect(QRectF(self.rect().x(), self.rect().y(),
                     self.rect().width(), height))

    height = pyqtProperty(float, getHeight, setHeight)


class RulerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene(self)
        self.view = QGraphicsView(self.scene, self)
        self.view.setFrameShape(QGraphicsView.NoFrame)  # Remove frame
        self.view.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # self.view.setRenderHint(QPainter.Antialiasing)

        self.full_seg = 60
        self.medium_seg = self.full_seg/2
        self.small_seg = self.medium_seg/4

        pen = QPen(Qt.black, 2)
        segment_width, segment_height = 5, self.small_seg
        self.enc_min = 0
        self.enc_max = 32_000
        self.num_segments = 40
        for i in range(self.num_segments):
            # color_ratio = i / (num_segments - 1)
            # color = QColor.fromRgbF(color_ratio, 1.0 - color_ratio, 0)
            color_ratio = abs(i - self.num_segments // 2) / (self.num_segments // 2)  # Red to yellow to green to yellow to red
            if color_ratio <= 0.5:
                color = QColor.fromRgbF(color_ratio * 2, 1.0, 0)
            else:
                color = QColor.fromRgbF(1.0, 1.0 - (color_ratio - 0.5) * 2, 0)
            
            segment = RulerSegment(
                i * (segment_width + 10), 0, segment_width, segment_height, color, i)
            segment.setPen(pen)
            self.scene.addItem(segment)
            self.setWindowFlags(Qt.FramelessWindowHint)  # Remove window frame
            # Set transparent background
            self.setAttribute(Qt.WA_TranslucentBackground)

        # Make the background behind the segments transparent
        self.view.setAutoFillBackground(False)
        self.view.setStyleSheet("background: transparent;")
        # Add a slider to control animation speed
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(self.enc_min)
        self.slider.setMaximum(self.enc_max)
        self.slider.setValue(self.enc_max/2)  # Initial animation duration
        self.slider.valueChanged.connect(self.update_height_animation)

        layout = QVBoxLayout()
        layout.addWidget(self.view)
        layout.addWidget(self.slider)
        self.setLayout(layout)

    def update_height_animation(self):
        # Update the animation duration based on the slider value
        
        val = int(self.slider.value()*(self.num_segments/self.enc_max))
        for segment in self.scene.items():
            if isinstance(segment, RulerSegment):
                try:
                    if segment.val == val:
                        segment.animate_height(self.full_seg)
                    elif segment.val == val + 1:
                        segment.animate_height(self.medium_seg)
                    elif segment.val == val - 1:
                        segment.animate_height(self.medium_seg)
                    else:
                        segment.animate_height(self.small_seg)
                except Exception as e:
                    print(e)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RulerApp()
    window.show()
    sys.exit(app.exec_())

