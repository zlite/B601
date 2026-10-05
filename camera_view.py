"""Upright presentation of the upside-down wrist camera; sensor data stays original."""
import cv2

VIEW_ROTATION_DEG = 180


def upright(frame):
    return cv2.rotate(frame, cv2.ROTATE_180)
