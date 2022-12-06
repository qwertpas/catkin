#!/usr/bin/env python

import copy
import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = (442-214)/(0.30) #pixels per meter or (447-232)/30 = 760
# tx = 231
tx=214
#222-tx)/760=0.205
# ty = 81
# 328-tx/760=0.15
ty = 66

# Function that converts image coord to world coord
def IMG2W(xy):
    col, row = xy

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

    # image_raw = cv2.bilateralFilter(image_raw,9,75,75)

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
        'teal': ((80,120,80), (130,255,255)),
        'pink': ((0, 100, 100), (50,255,255), (170, 100, 100), (180,255,255))
    }

    mask_dict = {}
    blob_dict = {}

    for color in color_dict:

        blob_dict[color] = []

        # Define a mask using the lower and upper bounds of the target color
        lower = color_dict[color][0]
        upper = color_dict[color][1]
        mask_image = cv2.inRange(hsv_image, lower, upper)
        if(color=='pink'):
            mask_image = mask_image + cv2.inRange(hsv_image, color_dict[color][2], color_dict[color][3])
        keypoints = detector.detect(mask_image)
        mask_dict[color] = mask_image

        # save blob centers in the image coordinates
        for i in range(len(keypoints)):
            if(keypoints[i].size > 15):
                blob_dict[color].append(np.array([keypoints[i].pt[0], keypoints[i].pt[1]]))
        num_blobs = len(blob_dict[color])

        # Draw the keypoints on the detected block
        for i in range(num_blobs):
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[i]], np.array([]), (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            # print(color, blob_dict[color][i])
            #convert image position to real world coordiantes
            blob_dict[color][i] = IMG2W(blob_dict[color][i])

    # for color in color_dict:
    #     cv2.imshow(f"Mask View {color}", mask_dict[color])
    # cv2.namedWindow("Keypoint View")
    # cv2.imshow("Keypoint View", im_with_keypoints)

    # if cv2.waitKey(1)& 0xFF == ord('q'):
    #     return blob_dict

    return blob_dict, mask_dict, im_with_keypoints
