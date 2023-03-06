# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimalist controller example for the Robot Wrestling Tournament.
   Demonstrates how to play a simple motion file."""

from controller import Robot, DistanceSensor

import sys,cv2,time
import numpy as np

# We provide a set of utilities to help you with the development of your controller. You can find them in the utils folder.
# If you want to see a list of examples that use them, you can go to https://github.com/cyberbotics/wrestling#demo-robot-controllers
sys.path.append('..')
from utils.motion_library import MotionLibrary
from utils.fall_detection import FallDetection
from utils.camera import Camera
from utils.image_processing import ImageProcessing as IP
from utils.current_motion_manager import CurrentMotionManager
    
def view2img(view,fname):
    # Extract the RGB channels and merge them into a single image
    bgr = view[:, :, :3]
    alpha = view[:, :, 3]
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgba = cv2.merge((rgb, alpha))

    # Save the image in RGB format
    cv2.imwrite(fname, rgba[:, :, :3])

class Wrestler (Robot):
    SMALLEST_TURNING_RADIUS = 0.
    TIME_BEFORE_DIRECTION_CHANGE = 200
    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())
        self.current_motion = CurrentMotionManager()
        #self.ipu=IP.ImageProcessing()
        # to load all the motions from the motions folder, we use the MotionLibrary class:
       
        self.motion= MotionLibrary()
        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        #self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 3.14 / 2
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0
        
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.time_step)
        self.sonarl = self.getDevice('Sonar/Left')
        self.sonarl.enable(self.time_step)
        self.sonarr = self.getDevice('Sonar/Right')
        self.sonarr.enable(self.time_step)
        
    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1
        
    def walk(self):
        """Walk towards the opponent like a homing missile."""
        normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
            self.heading_angle = - self.heading_angle
            self.counter = 0
        self.counter += 1
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)


    def run(self):
        
        #print(self.time_step)
        n=0
        
        #motion_library.play('TestMove')
        while self.step(self.time_step) != -1:  # mandatory function to make the simulation run
            value = [self.sonarl.getValue(),self.sonarr.getValue()]
            #print("Sonar value is: ", value)
            view=self.camera.get_image()
            #view2img(view,'v'+str(n)+'.png')
            _, _, horizontal_coordinate=IP.locate_opponent(view)
            #print(n,'\n',a[1],a[2])
            self.fall_detector.check()
            #if self.fall_detector.detect_fall():
            #    self.motion.play('TestMove')
            if self._get_normalized_opponent_x()<-.25:
                self.motion.play('ShiftLeft')
                #print('move left')
                #    self.motion.play('SideStepLeftLoop')
            elif self._get_normalized_opponent_x()>.25:
                self.motion.play('ShiftRight')
                    #self.motion.play('TestMove')
           
            #print(self.step(time_step))
            
                #print(self.current_motion.get())
            #    print(n)
            else:
                self.motion.play('MoveForward')
                
                #print(self.current_motion.get())
            #else:
                #print(n)
                #self.motion.play('Forwards')
                #self.walk()
            #self.motion.play('TestMove')
            n=n+1
            #time.sleep(5)


# create the Robot instance and run main loop
wrestler = Wrestler()
wrestler.run()
