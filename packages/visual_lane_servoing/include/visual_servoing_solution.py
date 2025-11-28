from typing import Tuple

import numpy as np
import cv2
from dt_computer_vision.ground_projection import GroundProjector
from dt_computer_vision.ground_projection.types import GroundPoint


# For vbot
LEFT_SCALING = -2.0
RIGHT_SCALING = 1.0

# For bobby
#  LEFT_SCALING = -5.0
#  RIGHT_SCALING = 1.0


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    width = shape[1] // 2
    height = shape[0]

    steer_matrix_left_lane = np.zeros((height, shape[1]))

    # Create ascending ramp: 0 → width-1
    steer_unit = np.arange(width, dtype=float)

    # Normalize (avoid divide-by-zero)
    max_val = steer_unit.max()
    if max_val != 0:
        steer_unit /= max_val

    # steer_matrix_left_lane[:, :width] = 1 # CHANGE ME
    steer_matrix_left_lane[:, :width] = LEFT_SCALING * np.ones((height, width)) * steer_unit

    return steer_matrix_left_lane


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    width = shape[1] // 2
    height = shape[0]

    # Prepare the output
    steer_matrix_right_lane = np.zeros((height, shape[1]))

    # Create descending ramp from width → 1
    steer_unit = np.arange(width, 0, -1).astype(float)

    # Normalize if not all zeros
    max_val = steer_unit.max()
    if max_val != 0:
        steer_unit /= max_val

    # steer_matrix_right_lane[:, width:] = 1 # CHANGE ME
    steer_matrix_right_lane[:, width:] = RIGHT_SCALING * np.ones((height, width)) * steer_unit # CHANGE ME

    return steer_matrix_right_lane


def detect_lane_markings(image: np.ndarray, projector: GroundProjector) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
        right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """

    # sigma = 8  # CHANGE ME - Gaussian blur sigma
    # threshold = 10  # CHANGE ME - minimum threshold for gradiant magnitude
    # white_lower_hsv = np.array([0, 0, 0])  # CHANGE ME - color thresholds
    # white_upper_hsv = np.array([179, 255, 255])  # CHANGE ME
    # yellow_lower_hsv = np.array([0, 0, 0])  # CHANGE ME
    # yellow_upper_hsv = np.array([179, 255, 255])  # CHANGE ME

    #  sigma = 5  # CHANGE ME - Gaussian blur sigma
    #  threshold = 50  # CHANGE ME - minimum threshold for gradiant magnitude
    #  white_lower_hsv = np.array([0, 0, 130])  # CHANGE ME 
    #  white_upper_hsv = np.array([160, 70, 255])  # CHANGE ME # 180, 70
    #  yellow_lower_hsv = np.array([20, 80, 100])  # CHANGE ME # very good
    #  yellow_upper_hsv = np.array([55, 255, 255])  # CHANGE ME

    #  white_lower_hsv = np.array([0, 0, 127])  # CHANGE ME 
    #  white_upper_hsv = np.array([179, 76, 255])  # CHANGE ME # 180, 70
    #  yellow_lower_hsv = np.array([21, 102, 100])  # CHANGE ME
    #  yellow_upper_hsv = np.array([63, 255, 255])  # CHANGE ME

    #  white_lower_hsv = np.array([0, 0, 127]) # 128
    #  white_upper_hsv = np.array([179, 77, 255])
    #  yellow_lower_hsv = np.array([77, 102, 102])
    #  yellow_upper_hsv = np.array([63, 255, 255])

    # For bobby
    sigma = 4  # CHANGE ME - Gaussian blur sigma
    threshold = 50  # CHANGE ME - minimum threshold for gradiant magnitude
    white_lower_hsv = np.array([0, 0, 127]) # 128
    white_upper_hsv = np.array([179, 77, 255])
    yellow_lower_hsv = np.array([21, 70, 70])  # CHANGE ME
    yellow_upper_hsv = np.array([63, 255, 255])  # CHANGE ME


    # for vbot
    sigma = 5  # CHANGE ME - Gaussian blur sigma
    threshold = 50  # CHANGE ME - minimum threshold for gradiant magnitude
    white_lower_hsv = np.array([np.int32(round(179/255 * 0)), np.int32(round(2.55 * 0)), np.int32(round(2.55 * 50))])         # CHANGE ME
    white_upper_hsv = np.array([np.int32(round(179/255 * 255)), np.int32(round(2.55 * 30)), np.int32(round(2.55 * 100))])   # CHANGE ME
    yellow_lower_hsv = np.array([np.int32(round(179/255 * 30)), np.int32(round(2.55 * 40)), np.int32(round(2.55 * 40))])        # CHANGE ME
    yellow_upper_hsv = np.array([np.int32(round(179/255 * 90)), np.int32(round(2.55 * 100)), np.int32(round(2.55 * 100))])  # CHANGE ME


    h, w, _ = image.shape

    imgbgr = image

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    imggray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)

    horizon = 270
    if projector is not None:
        far_away_point_ground = GroundPoint(x=10000000, y=0)
        normalized_vector = projector.ground2vector(far_away_point_ground)
        far_away_point_image = projector.camera.vector2pixel(normalized_vector)
        horizon = far_away_point_image.as_integers()[0]

    mask_ground = np.zeros((h, w), dtype=np.uint8)
    mask_ground[int(h - horizon + 50) :, :] = 1 # we add a small buffer to cut the entire horizon

    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(imggray, (0, 0), sigma)

    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 0, 1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx * sobelx + sobely * sobely)

    mask_mag = Gmag > threshold

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left = np.ones(sobelx.shape)
    mask_left[:, int(np.floor(w / 2)) : w + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:, 0 : int(np.floor(w / 2))] = 0

    mask_sobelx_pos = sobelx > 0
    mask_sobelx_neg = sobelx < 0
    mask_sobely_neg = sobely < 0

    mask_left_edge = mask_ground * mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge
