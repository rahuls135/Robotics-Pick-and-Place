#!/usr/bin/env python
import cv2
import numpy as np

def FindBlockByColor(image, color):
    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # lower = (hmin, smin, vmin)
    # upper = (hmax, smax, vmax)
    lower = (0,0,0)
    upper = (179,255,255)
    ##### Your Code Starts Here #####
    # ToDo: find and define lower and upper bounds of the target color
    # You can use either string or integer type for the 'color' input
    if color == 'red':
        lower = (0, 120, 70)
        upper = (10, 255, 255)
    elif color == 'yellow':
        lower = (20, 100, 100)
        upper = (30, 255, 255)
    elif color == 'blue':
        lower = (110, 50, 50)
        upper = (130, 255, 255)
    elif color == 'green':
        lower = (36, 25, 25)
        upper = (70, 255, 255)
    elif color == 'white':
        lower = (0, 0, 235)
        upper = (255, 20, 255)
    else:
        print("Invalid color")
    ##### Your Code Ends Here #####
    # Find mask image
    mask_image = cv2.inRange(hsv_image, lower, upper)
    # Remove comment signs to show the masked image result
    # cv2.namedWindow("Masked Image")
    # cv2.imshow("Masked Image", mask_image)
    # cv2.waitKey(0)
    # Output: a masked image for showing the position of the certain color block
    return mask_image

def FindBlockCenter(mask_image):
    _image = cv2.bitwise_not(mask_image)
    params = cv2.SimpleBlobDetector_Params()
    detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs
    keypoints = detector.detect(_image)
    _image = cv2.bitwise_not(_image)
    block_centers = []
    ##### Your Code Starts Here #####
    # ToDo: find blob centers in the image coordinates
    #
    # Output: store the results in 'block_centers'
    #
    # Hint: only 'keypoints' results are needed
    #
    # 'block_centers' array has a size of (n x 2), where n is the number of blobs
    found
    num_blobs = 0
    for i in keypoints:
        block_centers.append([int(keypoints[num_blobs].pt[0]), int(keypoints[num_blobs].pt[1])])
        num_blobs += 1
    ##### Your Code Ends Here #####
    # Remove comment signs to show the image with 'block_centers' result added
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    # im_with_keypoints = cv2.drawKeypoints(_image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # for i in range(num_blobs):
    #    cv2.circle(im_with_keypoints, (int(keypoints[i].pt[0]), int(keypoints[i].pt[1])), 2, (0, 0, 255), -1)
    # # Show keypoints
    # cv2.imshow("Keypoints", im_with_keypoints)
    # cv2.waitKey(0)
    return block_centers

def PixelToWorld(target_block_center_pixel, image):
    # ToDo: function that converts the detected center pixel position in image coord
    # to (x, y) coord in world frame
    #
    # Output: 'block_xy': x, y coordinates in world frame of a block, as a list
    #
    # Use the four white blocks as reference points to calculate coordinates
    block_xw = 0.0
    block_yw = 0.0
    block_xy = [block_xw, block_yw]
    ##### Your Code Starts Here #####
    # image = cv2.imread("../img_test_1.png")
    white_image = FindBlockByColor(image, 'white')
    conversionX, conversionY, coords = SplitWhiteIntoFour(white_image)
    block_xy[0] = -(target_block_center_pixel[0][1] - coords[0][1]) * conversionX
    block_xy[1] = -(target_block_center_pixel[0][0] - coords[0][0]) * conversionY - 0.5
    ##### Your Code Ends Here #####
    return block_xy

def FindColorBlockWorldCoord(image, color):
    # (Optional):
    # You may define the following function integrating all functions above
    # for easier usage in 'lab5_main.py' later
    block_xy = [0.0, 0.0]
    ##### Your Code Starts Here #####
    masked_img = FindBlockByColor(image, color)
    centers = FindBlockCenter(masked_img)
    block_xy = PixelToWorld(centers, image)
    block_xy = [round(i, 2) for i in block_xy]
    ##### Your Code Ends Here #####
    return block_xy
    # The above functions can be tested in the '__main__' function below:
    # Steps:
    # 1) Make this script executable
    # 2) Run 'python find_block_pos.py' in terminal

def SplitWhiteIntoFour(masked_img):
    (h,w) = masked_img.shape[:2]
    (cX,cY) = (w // 2, h // 2)
    topLeft = masked_img[0:cY, 0:cX]
    topRight = masked_img[0:cY, cX:w]
    bottomLeft = masked_img[cY:h, 0:cX]
    bottomRight = masked_img[cY:h, cX:w]

    topLeftCenters = FindBlockCenter(topLeft)
    topRightCenters = FindBlockCenter(topRight)
    bottomLeftCenters = FindBlockCenter(bottomLeft)
    bottomRightCenters = FindBlockCenter(bottomRight)

    topRightCenters[0][0] += cX
    bottomLeftCenters[0][1] += cY
    bottomRightCenters[0][1] += cY
    bottomRightCenters[0][0] += cX

    conversionX = (0.9 + 0.5)/(topRightCenters[0][0] - topLeftCenters[0][0])
    conversionY = (0.5 - 0)/(bottomRightCenters[0][1] - topRightCenters[0][1])

    return conversionX, conversionY, bottomRightCenters

if __name__ == '__main__':
    image = cv2.imread("../img_test_1.png")
    red_block = FindColorBlockWorldCoord(image, 'red')
    yellow_block = FindColorBlockWorldCoord(image, 'yellow')
    green_block = FindColorBlockWorldCoord(image, 'green')
    # blue_block = FindColorBlockWorldCoord(image, 'blue')
    print('red center', red_block)
    print('yellow center', yellow_block)
    print('green center', green_block)
    # print('blue center', blue_block)