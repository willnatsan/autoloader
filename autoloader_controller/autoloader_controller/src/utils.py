import cv2
import math
import numpy


"""
Skew Correction
"""


def find_skew(
    src_points: list[tuple[float, float]], dst_points: list[tuple[float, float]]
) -> list[list[float]]:

    src = numpy.array(src_points).astype(numpy.float32)
    dst = numpy.array(dst_points).astype(numpy.float32)

    return cv2.getAffineTransform(src, dst).tolist()


def correct_skew(
    x: float, y: float, skew: list[list[float]]
) -> tuple[float, float, float]:

    a, b, tx = skew[0]
    c, d, ty = skew[1]

    x_new = a * x + b * y + tx
    y_new = c * x + d * y + ty
    r_new = math.atan2(b, a)

    return round(x_new, 2), round(y_new, 2), round(r_new, 2)


"""
Runout Correction
"""


def kasa_circle_estimation(
    points: list[tuple[int, int]], rotation_angles: list[int]
) -> tuple[float, float, float]:
    # Reference: https://people.cas.uab.edu/~mosya/cl/CircleFitByKasa.cpp

    rms_error = 0.0
    mx = my = mxx = myy = mxy = mxz = myz = 0.0

    for point in points:
        mx += point[0]
        my += point[1]
    mx /= len(points)
    my /= len(points)

    for point in points:
        xi = point[0] - mx
        yi = point[1] - my
        zi = xi**2 + yi**2
        mxx += xi**2
        myy += yi**2
        mxy += xi * yi
        mxz += xi * zi
        myz += yi * zi
    mxx /= len(points)
    myy /= len(points)
    mxy /= len(points)
    mxz /= len(points)
    myz /= len(points)

    g11 = mxx**2
    g12 = mxy / g11
    g22 = (myy - g12**2) ** 2

    d1 = mxz / g11
    d2 = (myz - d1 * g12) / g22

    c = d2 / g22 / 2
    if c > 100:
        c = 0.0
    b = (d1 - c * g12) / g11 / 2

    center_x = b + mx
    center_y = c + my

    # Calculating Radius, RMS Error, and Phasing of Runout
    runout = b**2 + c**2 + mxx + myy
    if runout <= 0.0001:
        return 0.0, 0.0, 0.0

    for point in points:
        dx = point[0] - center_x
        dy = point[1] - center_y
        sq = (dx**2 + dy**2) ** 2 - runout
        rms_error += sq**2
    rms_error /= len(points)

    diff_prev = 0.0
    diff_total = 0.0
    for i in range(len(points)):
        diff = rotation_angles[i] - 180.0 / math.pi * math.atan2(
            points[i][1] - center_y, points[i][0] - center_x
        )
        if (diff_prev - diff) > 180:
            diff += 360
        elif (diff_prev - diff) < -180:
            diff -= 360

        diff_prev = diff
        diff_total += diff

    phasing = diff_total / len(points)
    if phasing > 180:
        phasing += 360
    elif phasing < -180:
        phasing -= 360

    return runout, phasing, rms_error
