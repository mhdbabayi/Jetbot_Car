#!/usr/bin/env python
import cv2
import numpy as np
import math
from datetime import datetime
import os

import time

# DIM=(1600, 1200)
# K=np.array([[781.3524863867165, 0.0, 794.7118000552183], [0.0, 779.5071163774452, 561.3314451453386], [0.0, 0.0, 1.0]])
# D=np.array([[-0.042595202508066574], [0.031307765215775184], [-0.04104704724832258], [0.015343014605793324]])
DIM = (640, 480)
K = np.array(
    [[401.85180238132534, 0.0, 315.0881937529724], [0.0, 534.1449296513911, 267.5899112187299], [0.0, 0.0, 1.0]])
D = np.array([[-0.044447931423351454], [0.09001009612247628], [-0.017793512771069935], [-0.25484017856839847]])

now = datetime.now()
current_time = now.strftime("%H_%M_%S")
filename = 'video_' + current_time + '.mp4'

frames_per_second = 24.0
res = '480p'


# Set resolution for the video capture
# Function adapted from https://kirr.co/0l6qmh
def change_res(cap, width, height):
    cap.set(3, width)
    cap.set(4, height)


# Standard Video Dimensions Sizes
STD_DIMENSIONS = {
    "480p": (640, 480),
    "720p": (1280, 720),
    "1080p": (1920, 1080),
    "4k": (3840, 2160),
}


def get_dims(cap, res='480p'):
    width, height = STD_DIMENSIONS["480p"]
    if res in STD_DIMENSIONS:
        width, height = STD_DIMENSIONS[res]
    ## change the current caputre device
    ## to the resulting resolution
    change_res(cap, width, height)
    return width, height


# Video Encoding, might require additional installs
# Types of Codes: http://www.fourcc.org/codecs.php
VIDEO_TYPE = {
    'avi': cv2.VideoWriter_fourcc(*'XVID'),
    'mp4': cv2.VideoWriter_fourcc(*'H264'),
    # 'mp4': cv2.VideoWriter_fourcc(*'XVID'),
}


def get_video_type(filename):
    filename, ext = os.path.splitext(filename)
    if ext in VIDEO_TYPE:
        return VIDEO_TYPE[ext]
    return VIDEO_TYPE['mp4']


class Detector:
    def __init__(self, dispW=800, dispH=600):
        flip = 0
        camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method=' + str(
            flip) + ' ! video/x-raw, width=' + str(dispW) + ', height=' + str(
            dispH) + ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
        self.cam = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.out = cv2.VideoWriter(filename, get_video_type(filename), 25, get_dims(self.cam, res))

        cv2.namedWindow('piCam')
        # createTrackbar(trackbarName,
        # windowName, value, count, onChange)
        # Brightness range -255 to 255
        cv2.createTrackbar('Brightness',
                           'piCam', 255, 2 * 255,
                           self.brightness_contrast)

        # Contrast range -127 to 127
        cv2.createTrackbar('Contrast', 'piCam',
                           127, 2 * 127,
                           self.brightness_contrast)

        self.missed_frames = 0

    @staticmethod
    def gstreamer_pipeline(
            capture_width=640,
            capture_height=480,
            display_width=640,
            display_height=480,
            framerate=60,
            flip_method=0,
    ):
        return (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
                )
        )

    @staticmethod
    def region_of_interest(img, vertices):
        mask = np.zeros_like(img)
        # channel_count = img.shape[2]
        match_mask_color = 255
        cv2.fillPoly(mask, vertices, match_mask_color)

        # mask = np.ones((480, 640))  # (height, width)
        # myROI = [(0,480), (320, 240), (325, 240), (640,480)]  # (x, y)
        # cv2.fillPoly(mask, [np.array(myROI)], 0)
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image

    @staticmethod
    def draw_the_lines(img, lines):
        img = np.copy(img)
        blank_image = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), thickness=5)

        img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
        return img

    @staticmethod
    def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                3
            ),
            dtype=np.uint8
        )
        img = np.copy(img)
        if lines is None:
            return
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img

    def brightness_contrast(self, img, brightness=0):

        # getTrackbarPos returns the current
        # position of the specified trackbar.
        brightness = cv2.getTrackbarPos('Brightness', 'piCam')

        contrast = cv2.getTrackbarPos('Contrast', 'piCam')

        effect = self.controller(img, brightness, contrast)

        # The function imshow displays an image
        # in the specified window
        # cv2.imshow('Effect', effect)
        return effect

    @staticmethod
    def controller(img, brightness=255, contrast=127):

        brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))

        contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

        if brightness != 0:

            if brightness > 0:

                shadow = brightness

                max = 255

            else:

                shadow = 0
                max = 255 + brightness

            al_pha = (max - shadow) / 255
            ga_mma = shadow

            # The function addWeighted calculates
            # the weighted sum of two arrays
            cal = cv2.addWeighted(img, al_pha,
                                  img, 0, ga_mma)

        else:
            cal = img

        if contrast != 0:
            Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
            Gamma = 127 * (1 - Alpha)

            # The function addWeighted calculates
            # the weighted sum of two arrays
            cal = cv2.addWeighted(cal, Alpha,
                                  cal, 0, Gamma)

            # putText renders the specified text string in the image.
        cv2.putText(cal, 'B:{},C:{}'.format(brightness,
                                            contrast), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return cal

    def line_grouping(self, lines, image):

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1)  # <-- Calculating the slope.
                if math.fabs(slope) < 0.5:  # <-- Only consider extreme slope
                    continue
                if slope <= 0:  # <-- If the slope is negative, left group.
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                else:  # <-- Otherwise, right group.
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])
        min_y = int(image.shape[0] * (3 / 5))  # <-- Just below the horizon
        max_y = image.shape[0]  # <-- The bottom of the image

        if len(left_line_x) == 0 or len(right_line_x) == 0:
            return None, None

        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))
        left_x_start = int(poly_left(max_y))  # m
        left_x_end = int(poly_left(min_y))
        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))
        centre = [int((left_x_start + right_x_start) / 2), max_y, int((left_x_end + right_x_end) / 2), min_y]
        line_image = self.draw_lines(
            image,
            [[
                [left_x_start, max_y, left_x_end, min_y], centre,
                [right_x_start, max_y, right_x_end, min_y],
            ]]
        )
        return line_image, centre

    def save_image(self):
        ret, frame = self.cam.read()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        now = datetime.now()

        current_time = now.strftime("%H_%M_%S")

        cv2.imwrite('Img_' + current_time + '.jpg', image)

    def detect_lanes(self):
        ret, frame = self.cam.read()
        cv2.imshow('piCam', frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        height = frame.shape[0]
        width = frame.shape[1]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # undistorted_img=frame
        region_of_interest_vertices = [
            (0, 480),
            (width / 2, 0),
            (640, 480)]

        gray_image = cv2.cvtColor(undistorted_img, cv2.COLOR_RGB2GRAY)
        # alpha = 2  # Contrast control (1.0-3.0)
        # beta = -200  # Brightness control (0-100)

        # adjusted = cv2.convertScaleAbs(gray_image, alpha=alpha, beta=beta)
        adjusted = self.brightness_contrast(gray_image, 0)
        canny_image = cv2.Canny(adjusted, 100, 200)

        cropped_image = self.region_of_interest(canny_image,
                                                np.array([region_of_interest_vertices], np.int32), )

        lines = cv2.HoughLinesP(cropped_image,
                                rho=6,
                                theta=np.pi / 180,
                                threshold=160,
                                lines=np.array([]),
                                minLineLength=40,
                                maxLineGap=25)
        if lines is None:
            cv2.imshow('piCam', undistorted_img)
            self.missed_frames += 1
            print("missed frame: lines ", self.missed_frames)
            return None
        # print("lines ",lines)
        # if lines:
        # image_with_lines = self.draw_the_lines(frame, lines)

        image_with_lines, centre = self.line_grouping(lines, undistorted_img)
        if image_with_lines is None:
            cv2.imshow('piCam', undistorted_img)
            self.missed_frames += 1
            print("missed frame: line_grouping ", self.missed_frames)
            return None

        cv2.imshow('piCam', image_with_lines)
        # self.out.write(image_with_lines)
        # cv2.imshow('piCam_crop', frame)
        return centre

    @staticmethod
    def get_lane_distance(line):
        if line is None:
            return None
        # bottom point
        x1 = line[0]
        y1 = line[1]
        # Top point (centre)
        x2 = line[2]
        y2 = line[3]

        error_distance = -(320 - (x1 + x2) / 2)
        num = y2 - y1
        if num > 0:
            den = x2 - x1
        else:
            num = y1 - y2
            den = x1 - x2
        if den is not 0:
            slope = num / den
            heading = math.degrees(math.atan(slope))
            # if abs(heading)>90:
            sign = -heading / abs(heading)
            heading = abs(90 - (abs(heading))) * sign
        else:
            heading = None
        return error_distance, heading

    def close(self):
        self.cam.release()
        self.out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    detector_obj = Detector()
    # detector_obj.tick()

    while True:
        centre = detector_obj.detect_lanes()
        if centre is not None:
            error_distance, heading = detector_obj.get_lane_distance(centre)
            print("lines ", centre, " Error ", error_distance, " Heading ", heading)
            # ros publish data
        # print("Error ",x)
        key = cv2.waitKey(1)
        if key == ord('c'):
            detector_obj.save_image()
        if key == ord('q'):
            break

    detector_obj.close()
