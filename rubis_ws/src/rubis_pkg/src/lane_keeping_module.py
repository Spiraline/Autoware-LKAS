#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
import sys
import os
import ctypes

from sensor_msgs.msg import CompressedImage
from autoware_msgs.msg import VehicleCmd

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

def find_firstfit(binary_array, direction, lower_threshold=40) :
    start, end = (binary_array.shape[0]-1, -1) if direction == -1 else (0, binary_array.shape[0])
    # print(start, end, direction)
    for i in range(start, end, direction) :
        if binary_array[i] > lower_threshold :
            return i
    return np.argmax(binary_array)

def color_gradient_transform(img, s_thresh=(90, 200), sx_thresh=(30, 100), r_thresh=(35,255)):
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
    
    # 320*240 ver
    x_mid = x_size/2
    top_margin = 30
    bottom_margin = 60
    top = 120
    bottom = 240
    bird_eye_margin = 30

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
    kernel = np.ones((5,5), np.float32) / 25
    dst = cv2.filter2D(binary_warped, -1, kernel)
    
    keep_center = 0
    out_img = np.dstack((dst, dst, dst))*255
    midpoint = np.int(dst.shape[1]/2)
    hj_window = 120
    leftx_base = find_firstfit(np.sum(dst[-hj_window:,:midpoint], axis=0), 1)
    rightx_base = find_firstfit(np.sum(dst[-hj_window:,midpoint:], axis=0), -1) + midpoint
    nwindows = 8
    window_height = np.int(dst.shape[0]/nwindows)
    
    nonzero = dst.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    # margin = 100
    margin = 30
    # Set minimum number of pixels found to recenter window
    minpix = 90

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Do we need labeling?
    # cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_warped)
    # print(centroids)
    # Step through the windows one by one
    left_cnt = 0
    right_cnt = 0
    state = 0 ## 0 is straight 1 is curve
    num = 0
    # for Left Lane
    for window in range(24):
        if left_cnt > 5:
            break
        # Identify window boundaries in x and y (and right and left)
        
        if state == 0:
            win_y_low = dst.shape[0] - (num+1)*window_height
            win_y_high = dst.shape[0] - num*window_height
            num += 1        
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
        
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        # print(nonzerox.shape, nonzeroy.shape)
        
        
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            # print('y value:', np.int(np.min(nonzeroy[good_left_inds])), np.int(np.max(nonzeroy[good_left_inds])))
            # print('x value:', np.int(np.min(nonzerox[good_left_inds])), np.int(np.max(nonzerox[good_left_inds])))
            min_x_idx = np.int(np.min(nonzerox[good_left_inds]))
            max_x_idx = np.int(np.max(nonzerox[good_left_inds]))
            min_y_idx = np.int(np.min(nonzeroy[good_left_inds]))
            max_y_idx = np.int(np.max(nonzeroy[good_left_inds]))
            # print(min_x_idx, max_x_idx, min_y_idx, max_y_idx, nonzerox[np.argmax(nonzeroy[good_left_inds])])
            # x_array = nonzerox[good_left_inds]
            # y_array = nonzeroy[good_left_inds]
            # slope = np.polyfit(x_array, y_array, 1)
            # slope_value = np.poly1d(slope)
            # print(len(good_left_inds))
            ## curve
            if max_x_idx - min_x_idx > 20:
                keep_center = 1
                #win_xleft_low = np.int(np.median(nonzerox[good_left_inds]))
                # good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
                leftx_current = np.int(np.median(nonzerox[good_left_inds]))
                
                
            ## straight
            else:    
                leftx_current = np.int(np.median(nonzerox[good_left_inds]))

        elif len(good_left_inds) < minpix:
            left_cnt += 1

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)

    state = 0
    num = 0
    # for Right Lane
    for window in range(24):
        if right_cnt > 5:
            break
        if state == 0:
            win_y_low = dst.shape[0] - (num+1)*window_height
            win_y_high = dst.shape[0] - num*window_height
            num += 1    
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        right_lane_inds.append(good_right_inds)

        if len(good_right_inds) > minpix:  
            min_x_idx = np.int(np.min(nonzerox[good_right_inds]))
            max_x_idx = np.int(np.max(nonzerox[good_right_inds]))
            if max_x_idx - min_x_idx > 20:
                rightx_current = np.int(np.median(nonzerox[good_right_inds]))
                keep_center = 1
            else:      
                rightx_current = np.int(np.median(nonzerox[good_right_inds]))
        elif len(good_right_inds) < minpix:
            right_cnt += 1

    left_line_err = 0
    right_line_err = 0

    # Concatenate the arrays of indices
    if len(left_lane_inds) > 0:
        left_lane_inds = np.concatenate(left_lane_inds)
    else:
        left_line_err = 1
    right_lane_inds = np.concatenate(right_lane_inds)
    
    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    try:
        linear_slope_left = np.polyfit(lefty, leftx, 1)
        # left_fit = np.polyfit(lefty, leftx, 2)
        
    except:
        left_line_err = 1
        
    try:
        linear_slope_right = np.polyfit(righty, rightx, 1)
        # right_fit = np.polyfit(righty, rightx, 2)

    except:
        right_line_err = 1
        

    slope_left = []
    slope_right = []
    slope_value = 0

    ## Left Lane Fail
    if left_line_err == 1 and right_line_err == 0:
        # slope_left =  np.poly1d(linear_slope_left)
        slope_right =  np.poly1d(linear_slope_right)
        slope_value = 200 - slope_right[0]

    ## Right Lane Fail
    elif left_line_err == 0 and right_line_err == 1:
        slope_left =  np.poly1d(linear_slope_left)
        # slope_right =  np.poly1d(linear_slope_left)
        slope_value = 80 - slope_left[0]
    
    elif left_line_err == 0 and right_line_err == 0:
        slope_left =  np.poly1d(linear_slope_left)
        slope_right =  np.poly1d(linear_slope_right)
        slope_value = 80 - slope_left[0]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    window_img = np.zeros_like(out_img)

    return out_img, slope_value

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
    sliding_window, sv = detect_lane_pixels(binary_warped)
    
    return binary_warped, sliding_window, warped, sv

class lane_keeping_module:
    
    def __init__(self):
        rospy.init_node('lane_keeping_module')
        # Should change to vehicle_cmd_lkas
        self.image_pub = rospy.Publisher('vehicle_cmd_lkas', VehicleCmd, queue_size = 1)
        self.image_sub = rospy.Subscriber('/simulator/camera_node/image/compressed',CompressedImage,self.callback)

    def lane_keeping_params(self, slope_value):
        velocity = 10
        
        angle = (slope_value)/100

        # print(angle)
        
        return velocity, angle

    def callback(self, data):
        start_time = timespec() ; end_time = timespec()
        clock_gettime(CLOCK_MONOTONIC_RAW , ctypes.pointer(start_time))
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imshow('cv_img', image_np)
        try:
            
            binary_warped, sliding_window, warped, sv = advanced_lane_detection_pipeline(image_np)
            # cv2.imshow('cv_bw', (binary_warped*255).astype(np.uint8))
            # cv2.imshow('cv_w', warped)
            # cv2.imshow('cv_sliding_w', sliding_window)
            
            cv2.waitKey(2)

            msg = VehicleCmd()
            velocity, angle = self.lane_keeping_params(sv)
            msg.twist_cmd.twist.linear.x = velocity
            msg.twist_cmd.twist.angular.z = angle 
            self.image_pub.publish(msg)
                

        except:
            msg = VehicleCmd()
            msg.twist_cmd.twist.linear.x = 5
            msg.twist_cmd.twist.angular.z = 0 
            self.image_pub.publish(msg)
            cv2.waitKey(2)
        
        clock_gettime(CLOCK_MONOTONIC_RAW , ctypes.pointer(end_time))

        print_file_path = os.getenv("HOME") + "/Documents/tmp/LKAS.csv"

        with open(print_file_path , "a") as f:
            f.write(f"{start_time.tv_sec + start_time.tv_nsec * 1e-9}, {end_time.tv_sec + end_time.tv_nsec * 1e-9}, {os.getpid()}\n")

        

def f(x, m, n):
    y = m*x + n
    return y

def main(args):
    ic = lane_keeping_module()

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
