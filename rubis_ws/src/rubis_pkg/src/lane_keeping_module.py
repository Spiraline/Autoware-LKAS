#!/usr/bin/env python3
import numpy as np
from numpy.lib.polynomial import poly1d
from numpy.linalg.linalg import slogdet
import rospy
import cv2
import sys
import platform
import pickle
import imutils
import glob
import matplotlib.image as mpimg
import rospkg
import os
import time
import ctypes

from sensor_msgs.msg import CompressedImage
from autoware_msgs.msg import VehicleCmd
from cv_bridge import CvBridge, CvBridgeError
np.set_printoptions(threshold=sys.maxsize)
prev_left_curv = None
prev_right_curv = None
prev_left_fitx = None
prev_right_fitx = None
cnt = 0

ym_per_pix = 30/720 # meters per pixel in y dimension
xm_per_pix = 3.7/70

prev_left_fit = None
prev_linear_left_fit = None

prev_right_fit = None
prev_linear_right_fit = None

__all__ = ["monotonic_time"]

CLOCK_MONOTONIC_RAW = 4 # see <linux/time.h>

class timespec(ctypes.Structure):
    _fields_ = [
        ('tv_sec', ctypes.c_long),
        ('tv_nsec', ctypes.c_long)
    ]

librt = ctypes.CDLL('librt.so.1', use_errno=True)
clock_gettime = librt.clock_gettime
clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(timespec)]

# Seonghyeon 
prev_pts_left = None
prev_pts_right = None

def find_firstfit(binary_array, direction, lower_threshold=20) :
    start, end = (binary_array.shape[0]-1, -1) if direction == -1 else (0, binary_array.shape[0])
    # print(start, end, direction)
    for i in range(start, end, direction) :
        if binary_array[i] > lower_threshold :
            return i
    return np.argmax(binary_array)

# we do not use ROI function 
def region_of_interest(img):
    height = img.shape[0]
    width = img.shape[1]
    polygons = np.array([
    [(400,height), (width-400, height), (width-400, 0),(400,0)]])
    mask = np.zeros_like(img)
    cv2.fillPoly(mask,polygons,(255,255,255))
    masked_img = cv2.bitwise_and(img,mask)
    return masked_img

def color_gradient_transform(img, s_thresh=(90, 255), sx_thresh=(30, 100), r_thresh=(35,100)):
    img = np.copy(img)
    
    R = img[:,:,0]
    G = img[:,:,1]
    B = img[:,:,2]
    
    # Threshold red channel
    rbinary = np.zeros_like(R)
    rbinary[(R >= r_thresh[0]) & (R <= r_thresh[1])] = 1
    # 
    # Convert to HLS color space and separate the s channel
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)#.astype(np.float)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    h_channel = hls[:,:,0]
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    
    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 0) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx)) #cv2.convertScaleAbs(abs_sobelx)

    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold saturation channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    # Stack each channel
    color_binary = np.dstack(( np.zeros_like(sxbinary), sxbinary, s_binary))
    
    combined_binary = np.zeros_like(sxbinary)
    # combined_binary[(s_binary == 1) | (sxbinary == 1)] = 1
    combined_binary[(s_binary == 1) | (sxbinary == 1) & (rbinary == 1)] = 1
    #combined_binary[(s_binary == 1) | (rbinary == 1)] = 1
    
    return color_binary, combined_binary

def warp(img):
    
    x_size = img.shape[1]
    y_size = img.shape[0]
    
    # set box boundary by magic values
    # x_mid = x_size/2
    # top_margin = 100
    # bottom_margin = 700
    # top = 370
    # bottom = 500
    # bird_eye_margin = 450

    # Complete Bird-eye Transform variables. 
    x_mid = x_size/2
    top_margin = 80
    bottom_margin = 420
    top = 390
    bottom = 550
    bird_eye_margin = 450

    # 4 Source coordinates
    src1 = [x_mid + top_margin, top] # top_right
    src2 = [x_mid + bottom_margin, bottom] # bottom_right
    src3 = [x_mid - bottom_margin, bottom] # bottom_left
    src4 = [x_mid - top_margin, top] # top_left
    src = np.float32([src1, src2, src3, src4])

    # 4 destination coordinates
    dst1 = [x_mid + bird_eye_margin, 0]
    dst2 = [x_mid + bird_eye_margin, y_size]
    dst3 = [x_mid - bird_eye_margin, y_size]
    dst4 = [x_mid - bird_eye_margin, 0]
    dst = np.float32([dst1, dst2, dst3, dst4])

    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    img_size = (img.shape[1], img.shape[0])
    warped = cv2.warpPerspective(img, M, img_size)
    
    # Return the resulting image and matrix
    return warped, M

def detect_lane_pixels(binary_warped):
    global prev_left_fit
    global prev_right_fit

    # Assuming you have created a warped binary image called "binary_warped"
    # Take a histogram of the bottom half of the image

    # histogram = np.sum(binary_warped[round(binary_warped.shape[0]/2):,:], axis=0)

    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    # midpoint = np.int(histogram.shape[0]/2)
    midpoint = np.int(binary_warped.shape[1]/2)
    
    # leftx_base = np.argmax(histogram[:midpoint])
    # rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    # binary_warped[:midpoint, 0]
    hj_window = 160
    # print(binary_warped[-hj_window:])
    leftx_base = find_firstfit(np.sum(binary_warped[-hj_window:,:midpoint], axis=0), -1)
    rightx_base = find_firstfit(np.sum(binary_warped[-hj_window:,midpoint:], axis=0), 1) + midpoint
    # print(leftx_base, rightx_base)    

    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int(binary_warped.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    maxpix = 800
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Do we need labeling?
    # cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_warped)
    # print(centroids)
    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        # cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
        # cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            #if len(good_left_inds) < maxpix:
            leftx_current = np.int(np.max(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            #if len(good_right_inds) < maxpix:
            rightx_current = np.int(np.min(nonzerox[good_right_inds]))


    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    try:
        linear_slope_left = np.polyfit(lefty, leftx, 1)
        left_fit = np.polyfit(lefty, leftx, 2)
        prev_left_fit = left_fit
        prev_linear_left_fit = linear_slope_left
    except TypeError as te:
        left_fit = prev_left_fit
        linear_slope_left = prev_linear_left_fit
    try:
        linear_slope_right = np.polyfit(righty, rightx, 1)
        right_fit = np.polyfit(righty, rightx, 2)
        prev_right_fit = right_fit
        prev_linear_right_fit = linear_slope_right
    except TypeError as te:
        right_fit = prev_right_fit
        linear_slope_right = prev_linear_right_fit

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    # out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    
    # Assume you now have a new warped binary image 
    # from the next frame of video (also called "binary_warped")
    # It's now much easier to find line pixels!
    # nonzero = binary_warped.nonzero()
    # nonzeroy = np.array(nonzero[0])
    # nonzerox = np.array(nonzero[1])
    # margin = 100
    # left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2] + margin))) 
    # right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2] + margin)))  

    # # Again, extract left and right line pixel positions
    # leftx = nonzerox[left_lane_inds]
    # lefty = nonzeroy[left_lane_inds] 
    # rightx = nonzerox[right_lane_inds]
    # righty = nonzeroy[right_lane_inds]

    # # Fit a second order polynomial to each
    
    # try:
    #     left_fit = np.polyfit(lefty, leftx, 2)
    #     prev_left_fit = left_fit
    # except TypeError as te:
    #     left_fit = prev_left_fit
    # try:
    #     right_fit = np.polyfit(righty, rightx, 2)
    #     prev_right_fit = right_fit
    # except TypeError as te:
    #     right_fit = prev_right_fit

    # # Generate x and y values for plotting
    # ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    # left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


    # Create an image to draw on and an image to show the selection window
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    # Color in left and right line pixels
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # Generate a polygon to illustrate the search window area
    # And recast the x and y points into usable format for cv2.fillPoly()
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    slope_left =  np.poly1d(linear_slope_left)
    slope_right =  np.poly1d(linear_slope_right)
    print(slope_left[0],slope_right[0])
    # = linear_slope_left.tolist()

    for idx in range(1000):
        slope_left_idx = slope_left(idx)
        slope_right_idx = slope_right(idx)
        cv2.line(out_img, (idx,int(slope_left_idx)),(idx,int(slope_left_idx)), (0,255,0), 5)
        cv2.line(out_img, (idx,int(slope_right_idx)),(idx,int(slope_right_idx)), (0,255,255), 5)

    return ploty, left_fit, right_fit, left_fitx, right_fitx, leftx, lefty, rightx, righty, out_img, slope_left[0], slope_right[0]
    
def determine_curvature(ploty, left_fit, right_fit, leftx, lefty, rightx, righty):
    global ym_per_pix
    global xm_per_pix
    # Define y-value where we want radius of curvature
    # I'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
    right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])
    # Define conversions in x and y from pixels space to meters
    # ym_per_pix = 30/720 # meters per pixel in y dimension
    # xm_per_pix = 3.7/700 # meters per pixel in x dimension

    # Fit new polynomials to x,y in world space
    # left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    # right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters

    return left_curverad, right_curverad

def labeling_lane(original_image):
    warped, M = warp(original_image)
    warped_color_binary, binary_warped = color_gradient_transform(warped)
    cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_warped)
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    for i in range(1,cnt):
        (x,y,w,h,area) = stats[i]
        print(area)
        if area < 10:
            continue
        cv2.rectangle(out_img, (x,y,w,h), (0,255,255))

    return out_img

def advanced_lane_detection_pipeline(original_image):
    global xm_per_pix
    global prev_left_curv
    global prev_right_curv
    global prev_left_fitx
    global prev_right_fitx
    global cnt
    
    warped, M = warp(original_image)
    warped_color_binary, binary_warped = color_gradient_transform(warped)
    # Detect Lane pixels
    ploty, left_fit, right_fit, left_fitx, right_fitx, leftx, lefty, rightx, righty, sliding_window, sl, sr = detect_lane_pixels(binary_warped)
    
    crossection = 0
    # Get curvature of each lane
    try:
        left_curverad, right_curverad = determine_curvature(ploty, left_fit, right_fit, leftx, lefty, rightx, righty)
        # Prevent
        curv_threshold = 5

        if cnt >= 1:
            if left_curverad < 90:
                left_fitx = prev_left_fitx
            if right_curverad < 90:
                right_fitx = prev_right_fitx
    except:
        crossection = 1
        
    # Create an image to draw the lines on
    # warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    # color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    # Recast the x and y points into usable format for cv2.fillPoly()
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    # pts = np.hstack((pts_left, pts_right))

    if crossection == False:
        # Center offset
        calc_center = (pts_right[0][0][0] + pts_left[0][-1][0]) / 2
        center_offset = (calc_center - original_image.shape[1]/2)

    else:
        center_offset = 0

    # prevent disturb
    if cnt >= 1:
        if abs(center_offset) >= 3.00:
            left_fitx = prev_left_fitx
            right_fitx = prev_right_fitx

    prev_left_fitx = left_fitx
    prev_right_fitx = right_fitx
    cnt += 1
    
    # Draw the lane onto the warped blank image
    # cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
    
    # Matrix for unwarping
    # inverse_perspective_M = np.linalg.inv(M)

    # Warp the blank back to original image space using inverse perspective matrix (Minv)
    # img_size = (color_warp.shape[1], color_warp.shape[0])
    # newwarp = cv2.warpPerspective(color_warp, inverse_perspective_M, img_size) 
    
    # Find center offset
    
    # Combine the result with the original image
    # result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)    
    result = original_image
    # font = cv2.FONT_HERSHEY_COMPLEX

    # cv2.putText(result, 'Left_curvrad: %.2f m'%(left_curverad), (30, 60), font, 1, (255,255,0), 2)
    # cv2.putText(result, 'Right_curvrad: %.2f m'%(right_curverad), (30, 90), font, 1, (255,255,0), 2)
    # cv2.putText(result, 'Center offset: %.2f '%(center_offset), (30, 120), font, 1, (255,255,0), 2)
    
    # Debug
    return result, center_offset, binary_warped, sliding_window, warped, sl, sr

class lane_keeping_module:

    def __init__(self):
        self.image_pub = rospy.Publisher("/vehicle_cmd_lkas",VehicleCmd, queue_size = 1)
        self.image_sub = rospy.Subscriber('/simulator/camera_node/image/compressed',CompressedImage,self.callback)

    def lane_keeping_params(self, center_offset, slope_left, slope_right):
        velocity = 5
        # angle = center_offset * (-1) / 20
        angle = 0
        
        # base slope values are 390, 869 respectively
        b_slope_left = 390
        b_slope_right = 869
        if slope_left:
            angle = - (slope_left - b_slope_left)/1000
        elif slope_right:
            angle = (slope_right - b_slope_right)/1000  
        print(angle)
        # if angle > 0.2:
        #     angle = 0.2
        # elif angle < -0.2:
        #     angle = -0.2
        
        # print(angle)
        # if center_offset > 0 :
        #     if center_offset > 30 :
        #         angle = - 0.5
        #     if center_offset > 20 :
        #         angle = - 0.2
        # elif center_offset < 0 : 
        #     if center_offset < -30 :
        #         angle = 0.5
        #     if center_offset < -20 :
        #         angle = 0.2
        # else :
        #     angle = 0
        
        # if angle == 0:
        #     velocity = 5
        # else:
        #     velocity = 2
        return velocity, angle

    def callback(self, data):
        start_time = timespec() ; end_time = timespec()
        clock_gettime(CLOCK_MONOTONIC_RAW , ctypes.pointer(start_time))
        
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # result = labeling_lane(image_np)
        # cv2.imshow('filtered', result) #(warp*255).astype(np.uint8 ))
        
        result, center_offset, binary_warped, sliding_window, warped, sl, sr = advanced_lane_detection_pipeline(image_np)
        # cv2.line(sliding_window, (640,0), (640,720), (255,0,0), 4)
        # cv2.line(sliding_window, (0,0), (1280,int(f(1280, 0.17054316,44.60857191))), (255,0,0), 4)
        
        # cv2.imshow('original_image', image_np)
        # cv2.imshow('bird-eye view', warped)
        # cv2.imshow('sliding_window', sliding_window)
        cv2.imshow('result_window', result)
        # cv2.imshow('filtered', (binary_warped*255).astype(np.uint8 )) #(warp*255).astype(np.uint8 ))
        cv2.waitKey(2)

        msg = VehicleCmd()
        velocity, angle = self.lane_keeping_params(center_offset, sl, sr)
        msg.twist_cmd.twist.linear.x = velocity
        msg.twist_cmd.twist.angular.z = angle 
        self.image_pub.publish(msg)

        clock_gettime(CLOCK_MONOTONIC_RAW , ctypes.pointer(end_time))

        print_file_path = os.getenv("HOME") + "/Documents/tmp/LKAS.csv"

        with open(print_file_path , "a") as f:
            f.write(f"{start_time.tv_sec + start_time.tv_nsec * 1e-9}, {end_time.tv_sec + end_time.tv_nsec * 1e-9}, {os.getpid()}\n")

def f(x, m, n):
    y = m*x + n
    return y

def main(args):
  ic = lane_keeping_module()
  rospy.init_node('lane_keeping_module')

  print_file_path = os.getenv("HOME") + "/Documents/tmp/LKAS.csv"

  with open(print_file_path , "w") as f:
      pass

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
