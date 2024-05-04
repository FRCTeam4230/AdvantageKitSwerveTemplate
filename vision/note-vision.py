import dataclasses
import math
import subprocess
import threading
import time
from typing import Sequence

import ntcore
from ntcore import NetworkTableInstance
from cscore import CameraServer

import numpy as np
import cv2

EXPOSURE_OPTIONS = [5, 10, 20, 39, 78, 156, 312, 625, 1250, 2500, 5000, 10000, 20000]
EXPOSURE_INDEX = 6

STREAM_FULL_RES = False

LIFECAM_RESOLUTION_OPTIONS = [
    (640, 480),
    (960, 544),
    (640, 360),
    (424, 240),
    (352, 288),
    (320, 240),
    (176, 144),
    (160, 120),
]
RESOLUTION = LIFECAM_RESOLUTION_OPTIONS[0]
RESOLUTION_DIAGONAL = math.hypot(RESOLUTION[0], RESOLUTION[1])
STREAM_RESOLUTION = (150, 100)

LIFECAM_DIAGONAL_FOV = 68.5


def get_axis_fov(size: int):
    return 2 * math.degrees(math.atan(
        math.tan(math.radians(LIFECAM_DIAGONAL_FOV / 2)) *
        size / RESOLUTION_DIAGONAL
    ))


LIFECAM_FOV_DEG = tuple(map(get_axis_fov, RESOLUTION))
CENTER_FRAME = tuple(map(lambda n: n / 2, RESOLUTION))
PIXEL_DISTANCE = (
    CENTER_FRAME[0] / math.tan(math.radians(LIFECAM_FOV_DEG[0]) / 2),
    CENTER_FRAME[1] / math.tan(math.radians(LIFECAM_FOV_DEG[1]) / 2)
)


# print({"fov": LIFECAM_FOV_DEG, "center": CENTER_FRAME, "pixel distance": PIXEL_DISTANCE})


@dataclasses.dataclass
class CameraConfig:
    path: str
    name: str


CAMERA_PORT_PATHS_PI4 = {
    'usb3': {
        'top': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0',
        'bottom': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-video-index0',
    },
    'usb2': {
        'top': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-video-index0',
        'bottom': '/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0',
    }
}

CAMERA_PORT_PATHS_PI5 = {
    'usb3': {
        'top': '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0',
        'bottom': '/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0',
    },
    'usb2': {
        'top': '/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0',
        'bottom': '/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0',
    },
}


CAMERA_CONFIGS = [
    CameraConfig(path=CAMERA_PORT_PATHS_PI5['usb3']['bottom'], name="center"),
    CameraConfig(path=CAMERA_PORT_PATHS_PI5['usb2']['bottom'], name="left"),
    CameraConfig(path=CAMERA_PORT_PATHS_PI5['usb2']['top'], name="right"),
]

HUE_SHIFT = 10
# Define lower and upper bounds for orange color in HSV
lower_orange_hsv = np.array([12, 150, 150])
upper_orange_hsv = np.array([25, 255, 255])
# The minimum contour area to detect a note
MINIMUM_CONTOUR_AREA = 500
# The threshold for a contour to be considered a disk
CONTOUR_DISK_THRESHOLD = 0.9
# how far down the image the contour would have to be in order to accept it if it is touching an edge
EDGE_Y_THRESHOLD = 0.10 * RESOLUTION[1]
# how many pixels the note can occupy
MAX_CONTOUR_AREA = 0.20 * RESOLUTION[0] * RESOLUTION[1]

NT_SERVER_MODE = False
NT_CONNECT_TO_SIM = False

if NT_CONNECT_TO_SIM:
    NetworkTableInstance.getDefault().setServer("192.168.7.233")
else:
    NetworkTableInstance.getDefault().setServerTeam(4230)

table = NetworkTableInstance.getDefault().getTable("note-vision")
config_table = table.getSubTable("config")
lower_config_table = config_table.getSubTable("min")
upper_config_table = config_table.getSubTable("max")


def publish_default_configs():
    lower_config_table.getEntry("h").setInteger(lower_orange_hsv[0]),
    lower_config_table.getEntry("s").setInteger(lower_orange_hsv[1]),
    lower_config_table.getEntry("v").setInteger(lower_orange_hsv[2]),

    upper_config_table.getEntry("h").setInteger(upper_orange_hsv[0]),
    upper_config_table.getEntry("s").setInteger(upper_orange_hsv[1]),
    upper_config_table.getEntry("v").setInteger(upper_orange_hsv[2]),

    config_table.getEntry("ellipse threshold").setDouble(CONTOUR_DISK_THRESHOLD)
    config_table.getEntry("exposure").setInteger(EXPOSURE_INDEX)




def setup_nt_listeners():
    listen_for_exposure()
    listen_for_hsv()


def listen_for_exposure():
    def listener(event: ntcore.Event):
        new_exposure = event.data.value.getInteger()
        for conf in CAMERA_CONFIGS:
            set_exposure(conf.path, new_exposure)

    entry = config_table.getEntry("exposure")

    NetworkTableInstance.getDefault().addListener(
        entry,
        ntcore.EventFlags.kValueRemote,
        listener
    )


def listen_for_hsv():
    def update_thresholds(_, __, ___):
        global lower_orange_hsv
        lower_orange_hsv = np.array([
            lower_config_table.getEntry("h").getInteger(lower_orange_hsv[0]),
            lower_config_table.getEntry("s").getInteger(lower_orange_hsv[1]),
            lower_config_table.getEntry("v").getInteger(lower_orange_hsv[2]),
        ])

        global upper_orange_hsv
        upper_orange_hsv = np.array([
            upper_config_table.getEntry("h").getInteger(upper_orange_hsv[0]),
            upper_config_table.getEntry("s").getInteger(upper_orange_hsv[1]),
            upper_config_table.getEntry("v").getInteger(upper_orange_hsv[2]),
        ])

    lower_config_table.addListener(ntcore.EventFlags.kValueAll, update_thresholds)
    upper_config_table.addListener(ntcore.EventFlags.kValueAll, update_thresholds)




def set_exposure(path: str, exposure_index: int):
    try:
        subprocess.call(
            f"v4l2-ctl -d {path} -c auto_exposure=1 -c exposure_time_absolute={EXPOSURE_OPTIONS[exposure_index]}",
            shell=True)
    except:
        pass


def set_resolution(cap: cv2.VideoCapture):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])


def get_yaw_and_pitch_from_contour(contour: np.ndarray) -> (float, float):
    moment = cv2.moments(contour)
    center_x = int(moment["m10"] / moment["m00"])
    center_y = int(moment["m01"] / moment["m00"])

    yaw = math.atan((CENTER_FRAME[0] - center_x) / PIXEL_DISTANCE[0])
    pitch = math.atan((center_y - CENTER_FRAME[1]) / PIXEL_DISTANCE[1])

    return yaw, pitch


def rotate_hue(hsv: np.ndarray) -> np.ndarray:
    h, s, v = cv2.split(hsv)
    hnew = np.mod(h + HUE_SHIFT, 180).astype(np.uint8)
    return cv2.merge([hnew, s, v])


def find_contours(filtered_frame: np.ndarray) -> Sequence[np.ndarray]:
    # Find contours in the mask
    contours, _ = cv2.findContours(filtered_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        return contours


def drop_resolution(frame: np.ndarray) -> np.ndarray:
    if STREAM_FULL_RES:
        return frame
    return cv2.resize(frame, STREAM_RESOLUTION)


def is_contour_large_enough(contour: np.ndarray) -> bool:
    return cv2.contourArea(contour) > MINIMUM_CONTOUR_AREA
    # Makes sure the contour isn't some random small spec of noise


def is_contour_on_edge(contour: np.ndarray) -> bool:
    """
    ignores the top edge because those notes are far away
    :param contour:
    :return: True if the contour is on the edge of the frame
    """

    x, y, w, h = cv2.boundingRect(contour)

    below_edge_y_threshold = y > EDGE_Y_THRESHOLD

    return (
            below_edge_y_threshold and (
            x == 0 or
            x + w == RESOLUTION[0]
    ) or y + h == RESOLUTION[1]
    )


def get_contour_area(contour: np.ndarray, filtered_frame: np.ndarray) -> int:
    contour_mask = np.zeros_like(filtered_frame)

    cv2.drawContours(contour_mask, [contour], -1, (1,), thickness=cv2.FILLED)

    contour_pixels = cv2.bitwise_and(filtered_frame, contour_mask)

    return np.count_nonzero(contour_pixels)


def is_contour_too_large(contour: np.ndarray, filtered_frame: np.ndarray) -> bool:
    return get_contour_area(contour, filtered_frame) > MAX_CONTOUR_AREA


def contour_is_note(contour: np.ndarray, filtered_frame: np.ndarray) -> bool:
    """
    Checks if the contour is shaped like a note
    :param contour: the contour to check
    :param filtered_frame: the thresholded image that the contour came from
    :return: True if the contour is shaped like a note
    """

    if is_contour_too_large(contour, filtered_frame):
        return False

    if is_contour_on_edge(contour):
        return True

    # Gets the smallest convex polygon that can fit around the contour
    contour_hull = cv2.convexHull(contour)
    # Fits an ellipse to the hull, and gets its area
    try:  # this can sometimes throw an error if the hull has less than 5 points
        ellipse = cv2.fitEllipse(contour_hull)
    except:
        return False
    best_fit_ellipse_area = np.pi * (ellipse[1][0] / 2) * (ellipse[1][1] / 2)
    # Returns True if the hull is almost as big as the ellipse
    return cv2.contourArea(contour_hull) / best_fit_ellipse_area > config_table.getEntry("ellipse threshold").getDouble(
        CONTOUR_DISK_THRESHOLD)


def handle_camera(config: CameraConfig, output_entry: ntcore.NetworkTableEntry):
    print("making streams: " + config.name)
    raw_output_stream = CameraServer.putVideo(
        f"notes-{config.name}-raw",
        STREAM_RESOLUTION[1],
        STREAM_RESOLUTION[0]
    )
    filtered_output_stream = CameraServer.putVideo(
        f"notes-{config.name}-filtered",
        STREAM_RESOLUTION[1],
        STREAM_RESOLUTION[0]
    )
    processed_output_stream = CameraServer.putVideo(
        f"notes-{config.name}-processed",
        STREAM_RESOLUTION[1],
        STREAM_RESOLUTION[0]
    )

    print("opening camera: " + config.name)
    while True:
        cap = cv2.VideoCapture(config.path)

        if not cap.isOpened():
            print(f"camera {config.name} not connected")
            time.sleep(3)

        print(f"camera {config.name} connected")

        set_resolution(cap)
        set_exposure(config.path, EXPOSURE_INDEX)

        print(f"main processing loop: {config.name}")
        while True:
            ret, frame = cap.read()
            timestamp_seconds = time.time()
            if not ret:
                print(f"issue getting frame from camera {config.name}")
                break

            raw_output_stream.putFrame(drop_resolution(frame))

            # Converts from BGR to HSV
            frame_hsv = rotate_hue(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
            # Threshold the HSV image to get only orange colors
            mask = cv2.inRange(frame_hsv, lower_orange_hsv, upper_orange_hsv)
#             mask = cv2.erode(mask, np.ones((15,2), np.uint8), iterations=1)
            contours = find_contours(mask)

            angles = []

            if contours is not None:
                contours = list(filter(is_contour_large_enough, contours))
                for contour in contours:
                    if contour_is_note(contour, frame):
                        cv2.ellipse(frame, cv2.fitEllipse(contour), (255, 0, 255), 5)
                        angles.append(get_yaw_and_pitch_from_contour(contour))

                cv2.drawContours(frame, contours, -1, (0, 255, 0), 4)
                cv2.drawContours(frame, [cv2.convexHull(contour) for contour in contours], -1, (255, 0, 0), 3)

            # format for n detections, [n pitches..., n yaws..., latency s]
            output_entry.setDoubleArray(
                [a[1] for a in angles] +
                [a[0] for a in angles] +
                [time.time() - timestamp_seconds]
            )

            processed_output_stream.putFrame(drop_resolution(frame))
            filtered_output_stream.putFrame(drop_resolution(mask))


def main():
    if NT_SERVER_MODE:
        print('starting nt server')
        NetworkTableInstance.getDefault().startServer()
    else:
        print('connecting main process to robot')
        NetworkTableInstance.getDefault().startClient4("note-vision-main")

    publish_default_configs()
    setup_nt_listeners()

    print('making threads')
    threads = []
    for config in CAMERA_CONFIGS:
        thread = threading.Thread(
            target=handle_camera,
            args=(config, table.getEntry(config.name)),
            name=config.name
        )

        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
