#!/usr/bin/env python3
from doctest import debug_script
import numpy as np
import rospy
import rospkg
import cv2
import math
import yaml

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import VehicleCmd

X_M_PER_PIX = 3.7/70
Y_M_PER_PIX = 30/720 # meters per pixel in y dimension

def find_firstfit(binary_array, direction, lower_threshold=20) :
    start, end = (binary_array.shape[0]-1, -1) if direction == -1 else (0, binary_array.shape[0])
    for i in range(start, end, direction) :
        if binary_array[i] > lower_threshold :
            return i
    return np.argmax(binary_array)

def color_gradient_filter(img, filter_thr_dict):
    s_thresh = filter_thr_dict['saturation_thr']
    sx_thresh = filter_thr_dict['x_grad_thr']
    r_thresh = filter_thr_dict['red_thr']
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
    
    return combined_binary

def birdeye_warp(img, birdeye_warp_param):
    x_size = img.shape[1]
    y_size = img.shape[0]
    x_mid = x_size/2

    # Tunable Parameter
    top = birdeye_warp_param['top']
    bottom = birdeye_warp_param['bottom']
    top_margin = birdeye_warp_param['top_width']
    bottom_margin = birdeye_warp_param['bottom_width']
    birdeye_margin = birdeye_warp_param['birdeye_width']

    # 4 Source coordinates
    src1 = [x_mid + top_margin, top] # top_right
    src2 = [x_mid + bottom_margin, bottom] # bottom_right
    src3 = [x_mid - bottom_margin, bottom] # bottom_left
    src4 = [x_mid - top_margin, top] # top_left
    src = np.float32([src1, src2, src3, src4])

    # 4 destination coordinates
    dst1 = [x_mid + birdeye_margin, 0]
    dst2 = [x_mid + birdeye_margin, y_size]
    dst3 = [x_mid - birdeye_margin, y_size]
    dst4 = [x_mid - birdeye_margin, 0]
    dst = np.float32([dst1, dst2, dst3, dst4])

    # Given src and dst points, calculate the perspective transform matrix
    trans_matrix = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    img_size = (img.shape[1], img.shape[0])
    birdeye_warped = cv2.warpPerspective(img, trans_matrix, img_size)
    
    # Return the resulting image and matrix
    return birdeye_warped

def detect_lane_pixels(filtered_img):
    ##### Tunable parameter
    firstfit_window_height = 100
    windows_num = 24
    margin = 30 # Set the width of the windows +/- margin
    minpix = 10 # Set minimum number of pixels found to recenter window



    out_img = np.dstack((filtered_img, filtered_img, filtered_img))*255
    midpoint = np.int(filtered_img.shape[1]/2)
    leftx_base = find_firstfit(np.sum(filtered_img[-firstfit_window_height:,:midpoint], axis=0), 1)
    rightx_base = find_firstfit(np.sum(filtered_img[-firstfit_window_height:,midpoint:], axis=0), -1) + midpoint
    window_height = np.int(filtered_img.shape[0]/windows_num)
    
    nonzero = filtered_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    
    left_cnt = 0
    right_cnt = 0
    isCurve = False # TODO : Do we need this variable?
    num = 0
    # for Left Lane
    for window in range(windows_num):
        if left_cnt > 12:
            break
        # Identify window boundaries in x and y (and right and left)
        
        if not isCurve:
            win_y_low = filtered_img.shape[0] - (num+1)*window_height
            win_y_high = filtered_img.shape[0] - num*window_height
            num += 1
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
        
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            min_x_idx = np.int(np.min(nonzerox[good_left_inds]))
            max_x_idx = np.int(np.max(nonzerox[good_left_inds]))
            min_y_idx = np.int(np.min(nonzeroy[good_left_inds]))
            max_y_idx = np.int(np.max(nonzeroy[good_left_inds]))
            ## curve
            if max_x_idx - min_x_idx > 20:
                leftx_current = np.int(np.median(nonzerox[good_left_inds]))
            ## straight
            else:    
                leftx_current = np.int(np.median(nonzerox[good_left_inds]))

        elif len(good_left_inds) < minpix:
            left_cnt += 1

        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)

    isCurve = False
    num = 0
    # for Right Lane
    for window in range(windows_num):
        if right_cnt > 12:
            break
        if not isCurve:
            win_y_low = filtered_img.shape[0] - (num+1)*window_height
            win_y_high = filtered_img.shape[0] - num*window_height
            num += 1
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        right_lane_inds.append(good_right_inds)

        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.median(nonzerox[good_right_inds]))
        elif len(good_right_inds) < minpix:
            right_cnt += 1

    left_line_err = 0
    right_line_err = 0

    ### Concatenate the arrays of indices
    if len(left_lane_inds) > 0:
        left_lane_inds = np.concatenate(left_lane_inds)
    else:
        left_line_err = 1
    right_lane_inds = np.concatenate(right_lane_inds)
    
    ### Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds] # x
    lefty = nonzeroy[left_lane_inds] # y
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    ### Fit a first order polynomial to each
    try:
        linear_slope_left = np.polyfit(leftx, lefty, 1)[0]*-1
        left_lane_angle = math.degrees(math.atan(linear_slope_left))
    except:
        left_line_err = 1

    try:
        linear_slope_right = np.polyfit(rightx, righty, 1)[0] * -1        
        right_lane_angle = math.degrees(math.atan(linear_slope_right))
    except:
        right_line_err = 1

    ### Generate x and y values for plotting
    angle_value = 0

    # Left Lane Fail (Only Right Lane detected)
    if left_line_err == 1 and right_line_err == 0:
        angle_value = right_lane_angle
        if angle_value > 0:
            angle_value = 0
            
    # Right Lane Fail (Only Left Lane detected)
    elif left_line_err == 0 and right_line_err == 1:
        angle_value = left_lane_angle
        if angle_value < 0:
            angle_value = 0
    
    # If two Lane are detected, see right lane
    elif left_line_err == 0 and right_line_err == 0:
        angle_value = right_lane_angle

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # print("# SLOPE VALUE:",angle_value)

    return out_img, angle_value # out_image = sliding window image
    
def determine_curvature(ploty, left_fit, right_fit, leftx, lefty, rightx, righty):
    global X_M_PER_PIX
    global Y_M_PER_PIX
    # Define y-value where we want radius of curvature
    # I'll choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)
    left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
    right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

    # Fit new polynomials to x,y in world space
    left_fit_cr = np.polyfit(lefty*Y_M_PER_PIX, leftx*X_M_PER_PIX, 2)
    right_fit_cr = np.polyfit(righty*Y_M_PER_PIX, rightx*X_M_PER_PIX, 2)
    # Calculate the new radii of curvature
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*Y_M_PER_PIX + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*Y_M_PER_PIX + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters

    return left_curverad, right_curverad

class lane_keeping_module:
    def __init__(self, config_dict):
        output_topic = rospy.get_param("/lkas/output_topic", "twist_cmd")
        debug_from_param = rospy.get_param("/lkas/debug_window", True)
        self.twist_pub = rospy.Publisher(output_topic, VehicleCmd, queue_size = 10)
        # self.twist_pub = rospy.Publisher(output_topic, TwistStamped, queue_size = 10)
        self.filter_thr_dict = config_dict['filter_thr_dict']
        self.birdeye_warp_param = config_dict['birdeye_warp_param']

        self.velocity = config_dict['velocity']
        self.steer_sensitivity = config_dict['steer_sensitivity']
        self.debug_window = config_dict['debug_window'] and debug_from_param

        if self.debug_window:
            self.trackbar_img = np.zeros((1,400), np.uint8)
            # Create Window
            cv2.namedWindow('original_image')
            cv2.namedWindow('birdeye_image')
            cv2.namedWindow('sliding_window')
            cv2.namedWindow('filtered_birdeye')
            cv2.namedWindow('TrackBar')

            # Move Window Location              #col    #row
            cv2.moveWindow('original_image',    350*0,  350*0)
            cv2.moveWindow('birdeye_image',     350*0,  350*1)
            cv2.moveWindow('sliding_window',    350*1,  350*0)
            cv2.moveWindow('filtered_birdeye',  350*1,  350*1)
            cv2.moveWindow('TrackBar',          350*2,  350*0)

            # Create Trackbar
            cv2.createTrackbar("[clr]sat_min", "TrackBar", self.filter_thr_dict['saturation_thr'][0], 255, self.onChange)
            cv2.createTrackbar("[clr]sat_max", "TrackBar", self.filter_thr_dict['saturation_thr'][1], 255, self.onChange)
            cv2.createTrackbar("[clr]red_min", "TrackBar", self.filter_thr_dict['red_thr'][0], 255, self.onChange)
            cv2.createTrackbar("[clr]red_max", "TrackBar", self.filter_thr_dict['red_thr'][1], 255, self.onChange)
            cv2.createTrackbar("[be]top", "TrackBar", self.birdeye_warp_param['top'], 240, self.onChange)
            cv2.createTrackbar("[be]bottom", "TrackBar", self.birdeye_warp_param['bottom'], 240, self.onChange)
            cv2.createTrackbar("[be]top_width", "TrackBar", self.birdeye_warp_param['top_width'], 320, self.onChange)
            cv2.createTrackbar("[be]bottom_width", "TrackBar", self.birdeye_warp_param['bottom_width'], 320, self.onChange)
            cv2.createTrackbar("[be]birdeye_width", "TrackBar", self.birdeye_warp_param['birdeye_width'], 320, self.onChange)

    def onChange(self, pos):
        pass

    def config_image_source(self, mode='webcam'):
        if mode == 'webcam':
            # VideoCapture(n) : n th input device (PC : 0, minicar : 1)
            self.capture = cv2.VideoCapture(1)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        elif mode == 'video':
            video_file = './test_video.avi'
            img = cv2.imread('test2.png', cv2.IMREAD_COLOR)
            width = 320
            height = 240
            dsize = (width, height)
            self.capture = cv2.VideoCapture(video_file)
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def calculate_velocity_and_angle(self, angle_value):
        velocity = self.velocity
        if angle_value > 0:
            target_angle = 90 - angle_value
        elif angle_value < 0:
            target_angle = -90 - angle_value
        else:
            target_angle = 0

        target_angle = target_angle * self.steer_sensitivity * -1
        
        return velocity, target_angle

    def svl_spinner(self):
        self.image_sub = rospy.Subscriber('/simulator/camera_node/image/compressed', CompressedImage, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self.debug_window:
            self.filter_thr_dict['saturation_thr'][0] = cv2.getTrackbarPos("[clr]sat_min", "TrackBar")
            self.filter_thr_dict['saturation_thr'][1] = cv2.getTrackbarPos("[clr]sat_max", "TrackBar")
            self.filter_thr_dict['red_thr'][0] = cv2.getTrackbarPos("[clr]red_min", "TrackBar")
            self.filter_thr_dict['red_thr'][1] = cv2.getTrackbarPos("[clr]red_max", "TrackBar")
            self.birdeye_warp_param['top'] = cv2.getTrackbarPos("[be]top", "TrackBar")
            self.birdeye_warp_param['bottom'] = cv2.getTrackbarPos("[be]bottom", "TrackBar")
            self.birdeye_warp_param['top_width'] = cv2.getTrackbarPos("[be]top_width", "TrackBar")
            self.birdeye_warp_param['bottom_width'] = cv2.getTrackbarPos("[be]bottom_width", "TrackBar")
            self.birdeye_warp_param['birdeye_width'] = cv2.getTrackbarPos("[be]birdeye_width", "TrackBar")

        cv2.waitKey(1)

        birdeye_image = birdeye_warp(image_np, self.birdeye_warp_param)
        filtered_birdeye = color_gradient_filter(birdeye_image, self.filter_thr_dict)
        sliding_window, angle_value = detect_lane_pixels(filtered_birdeye)

        if self.debug_window:
            cv2.imshow('TrackBar', self.trackbar_img)
            cv2.imshow('original_image', image_np)
            cv2.imshow('birdeye_image', birdeye_image)
            cv2.imshow('sliding_window', sliding_window)
            cv2.imshow('filtered_birdeye', (filtered_birdeye*255).astype(np.uint8))

        msg = TwistStamped()
        velocity, angle = self.calculate_velocity_and_angle(angle_value)

        if self.debug_window:
            print('-------------------------------')
            print('Angle : ', round(angle, 3))

        msg.twist.linear.x = velocity
        msg.twist.angular.z = angle
        vehicle_cmd_msg = VehicleCmd()
        vehicle_cmd_msg.twist_cmd = msg
        self.twist_pub.publish(vehicle_cmd_msg)

    def twist_publisher(self):
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            _, original_image = self.capture.read()

            if self.debug_window:
                self.filter_thr_dict['saturation_thr'][0] = cv2.getTrackbarPos("[clr]sat_min", "TrackBar")
                self.filter_thr_dict['saturation_thr'][1] = cv2.getTrackbarPos("[clr]sat_max", "TrackBar")
                self.filter_thr_dict['red_thr'][0] = cv2.getTrackbarPos("[clr]red_min", "TrackBar")
                self.filter_thr_dict['red_thr'][1] = cv2.getTrackbarPos("[clr]red_max", "TrackBar")
                self.birdeye_warp_param['top'] = cv2.getTrackbarPos("[be]top", "TrackBar")
                self.birdeye_warp_param['bottom'] = cv2.getTrackbarPos("[be]bottom", "TrackBar")
                self.birdeye_warp_param['top_width'] = cv2.getTrackbarPos("[be]top_width", "TrackBar")
                self.birdeye_warp_param['bottom_width'] = cv2.getTrackbarPos("[be]bottom_width", "TrackBar")
                self.birdeye_warp_param['birdeye_width'] = cv2.getTrackbarPos("[be]birdeye_width", "TrackBar")

            cv2.waitKey(1)

            birdeye_image = birdeye_warp(original_image, self.birdeye_warp_param)
            filtered_birdeye = color_gradient_filter(birdeye_image, self.filter_thr_dict)
            sliding_window, slope_value = detect_lane_pixels(filtered_birdeye)

            if self.debug_window:
                cv2.imshow('original_image', original_image)
                cv2.imshow('birdeye_image', birdeye_image)
                cv2.imshow('sliding_window', sliding_window)
                cv2.imshow('filtered_birdeye', (filtered_birdeye*255).astype(np.uint8))
                cv2.imshow('TrackBar', self.trackbar_img)

            msg = TwistStamped()
            velocity, angle = self.calculate_velocity_and_angle(slope_value)

            if self.debug_window:
                print('-------------------------------')
                print('Angle : ', round(angle, 3))

            msg.twist.linear.x = velocity
            msg.twist.angular.z = angle
            vehicle_cmd_msg = VehicleCmd()
            vehicle_cmd_msg.twist_cmd = msg
            self.twist_pub.publish(vehicle_cmd_msg)
            rate.sleep()
        
        self.capture.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        with open(rospkg.RosPack().get_path('lkas') + '/cfg/svl.yaml') as f:
            config_dict = yaml.load(f, Loader=yaml.FullLoader)
    except:
        print('Should make config file in config folder!')
        exit(1)

    rospy.init_node('lkas')

    # Mode : webcam, svl, video
    ic = lane_keeping_module(config_dict)
    if config_dict['mode'] == 'svl':
        ic.svl_spinner()
    else:
        ic.config_image_source(config_dict['mode'])
        ic.twist_publisher()

