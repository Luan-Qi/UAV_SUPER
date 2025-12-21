#!/usr/bin/env python3
# coding: utf-8
import os
import time
import rospy
import getpass
import threading
from astra_common import *
from geometry_msgs.msg import Twist
from transbot_msgs.msg import Position
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server

from astra_tracker.cfg import AstraColorHSVConfig
from dynamic_reconfigure.client import Client
class Astra_Tracker:
    def __init__(self):
        rospy.init_node("Astra_Tracker", anonymous=False)
        rospy.on_shutdown(self.cancel)
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT', 'color']
        self.tracker_type = rospy.get_param("~tracker_type", 'color')
        self.user_name = getpass.getuser()
        text_path = '/home/' + self.user_name + '/Desktop/UAV-Super/src/uav_tracker/script'
        self.hsv_text = text_path + "/AstraTrackerHSV.text"

        Server(AstraColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("/Astra_Tracker", timeout=60)
        print("OpenCV Version: ", cv.__version__)
        if self.tracker_type == "color":
            self.Track_state = 'identify'
        else:
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            self.tracker_type = self.tracker_types[self.index]
            self.Track_state = 'init'


    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        if action == ord('i') or action == ord('I'): self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): self.Reset()
        elif action == ord('q') or action == ord('Q'): self.cancel()
        elif action == ord('f') or action == ord('F'):
            self.index += 1
            if self.index >= len(self.tracker_types): self.index = 0
            self.tracker_type = self.tracker_types[self.index]
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            print("tracker_type: ", self.tracker_type)
            self.Reset()
            self.Track_state = 'init'
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    if self.tracker_type == "color": rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text) and self.tracker_type == "color":
                self.hsv_range = read_HSV(self.hsv_text)
            else: self.Track_state = 'init'
        if self.Track_state != 'init':
            if self.tracker_type == "color" and len(self.hsv_range) != 0:
                rgb_img, binary = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False
            if self.tracker_type != "color":
                if self.gTracker_state == True:
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                    self.gTracker = Tracker(tracker_type=self.tracker_type)
                    self.gTracker.initWorking(rgb_img, Roi)
                    self.gTracker_state = False
                rgb_img = self.gTracker.track(rgb_img)
        if self.tracker_type == "color": cv.putText(rgb_img, " color", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        else : cv.putText(rgb_img, self.tracker_type + " Tracker", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        return rgb_img, binary

    def cancel(self):
        self.Reset()
        self.dyn_client.close()
        cv.destroyAllWindows()

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        rospy.loginfo("init succes!!!")


if __name__ == '__main__':
    '''
    距离相机Astra的范围【0.4米，8米】，否则无效
    '''
    astra_tracker = Astra_Tracker()

    capture = cv.VideoCapture(0)
    cv_edition = cv.__version__
    if cv_edition[0] == '3': capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else: capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture: ",capture.isOpened())
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        action = cv.waitKey(10) & 0xFF
        frame, binary = astra_tracker.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        thread_text = "thread : " + str(len(threading.enumerate()))
        cv.putText(frame, thread_text, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)
        if len(binary) != 0:
            cv.imshow('frame', ManyImgs(1, ([frame, binary])))
        else:
            cv.imshow('frame', frame)
        if action == ord('q') or action == ord('Q'):
            astra_tracker.cancel()
            break
    capture.release()
    cv.destroyAllWindows()
