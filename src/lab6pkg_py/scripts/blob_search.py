#!/usr/bin/env python

import copy
import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = (439-236)/(0.28) #pixels per meter or (447-232)/30
tx = 232
ty = 63

# Function that converts image coord to world coord
def IMG2W(xy):
    row, col = xy

    x = (row - ty)/beta
    y = (col - tx)/beta

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    coord = R @ np.array([x, y])
    return coord

# ========================= Student's code ends here ===========================

def blob_search(image_raw):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False


    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    im_with_keypoints = copy.deepcopy(image_raw)

    color_dict = {
        'green': ((45,100,50), (80,255,255)),
        'yellow': ((10,150,120), (40,255,255)),
        'orange': ((0,150, 130), (25,255,255))
    }

    blob_dict = {}

    for color in color_dict:

        blob_dict[color] = []

        # Define a mask using the lower and upper bounds of the target color
        lower = color_dict[color][0]
        upper = color_dict[color][1]
        mask_image = cv2.inRange(hsv_image, lower, upper)

        keypoints = detector.detect(mask_image)

        # save blob centers in the image coordinates
        for i in range(len(keypoints)):
            if(keypoints[i].size > 15):
                blob_dict[color].append(np.array([keypoints[i].pt[0], keypoints[i].pt[1]]))
        num_blobs = len(blob_dict[color])

        # Draw the keypoints on the detected block
        for i in range(num_blobs):
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[i]], np.array([]), upper, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            #convert image position to real world coordiantes
            blob_dict[color][i] = IMG2W(blob_dict[color][i])


    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    if cv2.waitKey(1)& 0xFF == ord('q'):
        return blob_dict

    return blob_dict
