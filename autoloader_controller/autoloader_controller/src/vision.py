import cv2
import logging
import numpy
import threading
import time

from paddleocr import PaddleOCR

logger = logging.getLogger(__name__)

_ocr_model = PaddleOCR(
    use_angle_cls=True, lang="en", det_algorithm="DB", show_log=False
)


class Camera:
    def __init__(
        self,
        src: str,
        units_per_pixel_x: float,
        units_per_pixel_y: float,
        units_scale_factor: float = 1,
    ) -> None:
        self._capture = cv2.VideoCapture(src)

        self.image_width = 1280
        self.image_height = 720
        self._units_per_pixel = (
            units_per_pixel_x * units_scale_factor,
            units_per_pixel_y * units_scale_factor,
        )

        _ = self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        _ = self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        _ = self._capture.set(cv2.CAP_PROP_FPS, 5)

        self._status = False
        self._frame = None

        self._thread_running = True
        self._thread = threading.Thread(target=self.update)
        self._thread.daemon = True
        self._thread.start()

    def __del__(self) -> None:
        self.stop()

    def update(self) -> None:
        while self._thread_running:
            if self._capture.isOpened():
                self._status, self._frame = self._capture.read()
            time.sleep(0.01)

    def stop(self) -> None:
        self._thread_running = False
        self._thread.join()
        self._capture.release()

    def get_frame(self) -> tuple[bool, cv2.typing.MatLike]:
        return self._status, self._frame

    def get_pixels_offset(
        self, target_x: float, target_y: float
    ) -> tuple[float, float]:
        pixel_offset_x = target_x - self.image_width / 2
        pixel_offset_y = target_y - self.image_height / 2

        return pixel_offset_x, pixel_offset_y

    def get_mm_offset(self, target_x: float, target_y: float) -> tuple[float, float]:
        pixel_offset_x, pixel_offset_y = self.get_pixels_offset(target_x, target_y)

        # Flipping X-Offset due to the flipped top camera
        return (
            -pixel_offset_x * self._units_per_pixel[0],
            pixel_offset_y * self._units_per_pixel[1],
        )


def mask_image(image: cv2.typing.MatLike, radius: int) -> numpy.ndarray:
    mask = numpy.zeros((image.shape[0], image.shape[1]), dtype=numpy.uint8)
    _ = cv2.circle(
        mask,
        (int(image.shape[1] / 2), int(image.shape[0] / 2)),
        radius,
        (255, 255, 255),
        -1,
    )

    return cv2.bitwise_and(image, image, mask=mask)


def crop_image(image: cv2.typing.MatLike, range: int) -> numpy.ndarray:
    x0 = image.shape[1] / 2 - range
    x1 = image.shape[1] / 2 + range
    y0 = image.shape[0] / 2 - range
    y1 = image.shape[0] / 2 + range
    return image[int(y0) : int(y1), int(x0) : int(x1)]


def rotate_image(image: cv2.typing.MatLike, angle: float) -> cv2.typing.MatLike:
    image_center = tuple(numpy.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


def find_text(image: cv2.typing.MatLike) -> tuple[bool, str]:
    image_output = rotate_image(image, 45)
    image_blurred = cv2.GaussianBlur(image_output, (0, 0), 3)
    image_sharpened = cv2.addWeighted(image_blurred, 1.5, image_output, -0.5, 0)
    results = _ocr_model.ocr(image_sharpened)
    text = ""

    for result in results:
        if result is None:
            continue
        for line in result:
            (x1, y1), _, (x2, y2), _ = line[0]
            top_left = (int(x1), int(y1))
            bot_right = (int(x2), int(y2))
            text_location = (int(x1), int(y1) - 20)
            text = line[1][0]
            _ = cv2.rectangle(image_output, top_left, bot_right, (0, 255, 0), 4)
            _ = cv2.putText(
                image_output,
                text + f" [Accuracy: {line[1][1]:.3f}]",
                text_location,
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )

    if not text:
        logger.error("Unable to detect text in image")
        return False, text

    return True, text


def find_circular(
    image: cv2.typing.MatLike, radius: int, tolerance: int = 5
) -> tuple[bool, tuple[int, int]]:

    image_output = image.copy()
    image_gray = cv2.cvtColor(image_output, cv2.COLOR_BGR2GRAY)
    image_blurred = cv2.medianBlur(image_gray, 5)

    circle_center_x = 0
    circle_center_y = 0

    # I don't fully understand it myself, but the given parameters seem to result in the best circle detection
    # Thresholding works in theory but has been causing issues in practice. For now, just gray-scaling works great
    circles = cv2.HoughCircles(
        image_blurred,
        cv2.HOUGH_GRADIENT,
        1.01,
        20,
        param1=200,
        param2=20,
        minRadius=radius - tolerance,
        maxRadius=radius + tolerance,
    )
    if circles is None:
        logger.error("No valid circles detected")
        return False, (circle_center_x, circle_center_y)

    circles_ints = numpy.around(circles[0, :]).astype("int")
    circle_center_x, circle_center_y, radius = min(
        circles_ints, key=lambda circle: abs(circle[2] - radius)
    )

    _ = cv2.circle(
        image_output, (circle_center_x, circle_center_y), radius, (0, 255, 0), 4
    )
    _ = cv2.circle(
        image_output, (circle_center_x, circle_center_y), 2, (0, 128, 255), -1
    )

    return True, (circle_center_x, circle_center_y)


def find_rectangular(image: cv2.typing.MatLike) -> tuple[bool, tuple[int, int, float]]:
    image_output = image.copy()
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_blurred = cv2.GaussianBlur(image_gray, (5, 5), 0)
    image_thresholded = cv2.threshold(image_blurred, 220, 255, cv2.THRESH_BINARY)[1]

    max_area_curr = 0.0
    rectangle_center_x = 0
    rectangle_center_y = 0
    rectangle_rotation = 0.0

    contours, _ = cv2.findContours(
        image_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if contours is None:
        logger.error("No valid contours detected")
        return False, (rectangle_center_x, rectangle_center_y, rectangle_rotation)

    for contour in contours:
        approx_point_count = cv2.approxPolyDP(
            contour, 0.025 * cv2.arcLength(contour, True), True
        )
        if len(approx_point_count) == 4:
            try:
                (center_x, center_y), (width, height), angle = cv2.minAreaRect(contour)
                rectangle_points = cv2.boxPoints(
                    ((center_x, center_y), (width, height), angle)
                )
                rectangle_points = rectangle_points.astype("int")

                if abs(angle) > 45:
                    angle_reference = min(
                        (-180, -90, 90, 180), key=lambda a: abs(angle - a)
                    )
                    angle -= angle_reference

                if width * height > max_area_curr:
                    max_area_curr = width * height
                    rectangle_center_x = center_x
                    rectangle_center_y = center_y
                    rectangle_rotation = -angle

                    _ = cv2.drawContours(
                        image_output, [rectangle_points], 0, (255, 255, 0), 2
                    )
                    _ = cv2.circle(
                        image_output, (int(center_x), int(center_y)), 2, (0, 0, 255), -1
                    )

            except ZeroDivisionError:
                continue

    if not (rectangle_center_x or rectangle_center_y or rectangle_rotation):
        logger.error("No valid rectangles detected")
        return False, (rectangle_center_x, rectangle_center_y, rectangle_rotation)

    return True, (rectangle_center_x, rectangle_center_y, rectangle_rotation)
