#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0
beta = (439-236)/(0.28) #pixels per meter or (447-232)/30
tx = 232
ty = 63

# Function that converts image coord to world coord
def IMG2W(col, row):
    x = (row - ty)/beta
    y = (col - tx)/beta

    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    (x, y) = R @ np.array([x, y])
    return x, y

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

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

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    if color == 'green':
        lower = (45,100,50)     # green lower
        upper = (80,255,255)   # green upper
    elif color == 'yellow':
        lower = (10,150,120)     # yellow lower
        upper = (40,255,255)   # yellow upper
    else:
        #orange
        lower = (0,150, 130)     # orange lower
        upper = (25,255,255)   # orange upper

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        if(keypoints[i].size > 15):
            blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))
    num_blobs = len(blob_image_center)
    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    if(num_blobs != 0):
        if(color == 'green'):
            im_with_keypoints = cv2.drawKeypoints(image_raw, [keypoints[0]], np.array([]), (0,255,0))
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[0]], np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        elif(color == 'yellow'):
            im_with_keypoints = cv2.drawKeypoints(image_raw, [keypoints[0]], np.array([]), (0,255,255))
            im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[0]], np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            #calibration

            im_with_keypoints = image_raw

            if(len(keypoints) == 2):
                orange_pts = [[], []]
                for i in range(len(keypoints)):
                    if(keypoints[i].size > 15):
                        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[i]], np.array([]), (0,100,255))
                        im_with_keypoints = cv2.drawKeypoints(im_with_keypoints, [keypoints[i]], np.array([]), (0,100,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                        # print(f"orange {i}: {keypoints[i].pt}")
                        orange_pts[i] = keypoints[i].pt

                try:
                    if(orange_pts[0][0] > orange_pts[1][0]):
                        tmp = orange_pts[0]
                        orange_pts[0] = orange_pts[1]
                        orange_pts[1] = tmp
                    

                    betw_oranges = np.array(orange_pts[1]) - np.array(orange_pts[0])
                    pixel_dist = np.linalg.norm(betw_oranges) #
                    pixels_per_meter = pixel_dist / 0.10 #pixels per meter
                    print(f"pixels per meter: {pixels_per_meter}")

                    print(f"theta: {np.degrees(np.arctan2(betw_oranges[1], betw_oranges[0]))}")
                except:
                    print("no calibration target found")


    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        # print("No block found!")
        return []
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            # if(len(blob_image_center) > 0 and len(blob_image_center[0]) > 0):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))

    # print(xw_yw[0])

    # cv2.namedWindow("Camera View")
    # cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
