# Team: Paul Walker
# Authors: Jason ***, Ye ***, Konstantinos Alexopoulos
# Class: COMP/ELEC 424/553
# Final Project: Autonomous RC Car 
# Fall 2022

# Code drawn from:
# https://www.hackster.io/really-bad-idea/autonomous-path-following-car-6c4992

# TODO: cite traffic light detection ML model used (probably YOLO)

# TODO: 
#   run on AI-64 - might need to modify camera dims & add camera index
#   add PWM code to reset_car, start_car, turn_car
#   tune PD variables
#   manage speed in main loop using driver & speed encoder
#   insert stop sign detection & logic in main loop
#   writeup hackster page

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import time

# Camera
frame_width = 320
frame_height = 240

sightDebug = True

# PD variables
kp = 0.085
kd = kp * 0.1

# Throttle
throttlePin = "P8_13"
go_forward = 7.91
go_faster_addition = 0.022
go_faster_tick_delay = 80
go_faster_tick = 0  # Do not change this here. Code will set this value after seeing stop sign
dont_move = 7.5

# Steering
steeringPin = "P9_14"
left = 9
right = 6

# Max number of loops
max_ticks = 2000


def reset_car():
    # TODO: SET PWM TO 0 for steering and speed
    print("Car: stopped & straightened")
    

def start_car():
    # TODO: Set PWM to 7.5
    print("Car: ready (input something)")
    input()
    
def turn_car(turn_amt):
    # TODO: Turn CAR
    print("Car turn amount: ", turn_amt);
    
def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([90, 120, 0], dtype="uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    edges = cv2.Canny(mask, 50, 100)

    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    cropped_edges = cv2.bitwise_and(edges, mask)

    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape

    slope, intercept = line

    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi

    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape

    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle

def plot_pd(p_vals, d_vals, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    if show_img:
        plt.show()
    plt.clf()

def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed PWM")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering PWM")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PWM Values")
    ax2.set_ylabel("Error Value")

    plt.title("PWM Values over time")
    fig.legend()
    plt.savefig("pwm_plot.png")

    if show_img:
        plt.show()
    plt.clf()


def main():

    # set up video
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
    video.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
    
    reset_car()  
    
    # arrays for making the final graphs
    p_vals = []
    d_vals = []
    err_vals = []
    speed_pwm = []
    steer_pwm = []
    current_speed = go_forward
    
    # loop-related variables
    stopSignCheck = 1
    isStopSignBool = False
    counter = 0
    lastTime = 0
    lastError = 0
    
    start_car()
    
    while counter < max_ticks:
    
        # manage video
        ret, original_frame = video.read()
        frame = cv2.resize(original_frame, (160, 120))
        
        # process the frame to determine the desired steering angle
        edges = detect_edges(frame)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame, line_segments)
        lane_lines_image = display_lines(frame, lane_lines)
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        
        if sightDebug:
            cv2.imshow("heading line",heading_image)

        # calculate changes for PD
        now = time.time()
        dt = now - lastTime
        deviation = steering_angle - 90

        # PD Code
        error = -deviation
        base_turn = 7.5
        proportional = kp * error
        derivative = kd * (error - lastError) / dt

        # take values for graphs
        p_vals.append(proportional)
        d_vals.append(derivative)
        err_vals.append(error)

        # determine actual turn to do
        turn_amt = base_turn + proportional + derivative

        # caps turns to make PWM values
        if 7.2 < turn_amt < 7.8:
            turn_amt = 7.5
        elif turn_amt > left:
            turn_amt = left
        elif turn_amt < right:
            turn_amt = right

        turn_car(turn_amt) 
        
        # take values for graphs
        steer_pwm.append(turn_amt)
        speed_pwm.append(current_speed)
        
        # update PD values for next loop
        lastError = error
        lastTime = time.time()

        key = cv2.waitKey(1)
        if key == 27:
            break

        counter += 1
    
        
    # clean up resources
    reset_car()
    video.release()
    cv2.destroyAllWindows()
    plot_pd(p_vals, d_vals, err_vals, True)
    plot_pwm(speed_pwm, steer_pwm, err_vals, True)

if __name__ == "__main__":
    main()
