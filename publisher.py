#!/usr/bin/env python

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time

import numpy as np
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class converter:
    img1 = None
    img2 = None
    img3 = None
    img4 = None
    def __init__(self, point = None, f = 570):
        
        # reset node
        rospy.init_node('4_image_publisher')

        # setting topic
        self.pub1 = rospy.Publisher("image_topic_1", Image)
        self.pub2 = rospy.Publisher("image_topic_2", Image)
        self.pub3 = rospy.Publisher("image_topic_3", Image)
        self.pub4 = rospy.Publisher("image_topic_4", Image)

        # setting image size
        self.IM_WIDTH = 1024
        self.IM_HEIGHT = 1024

        # 4 sensors image will be saved here
        #self.img1 = None
        #self.img2 = None
        #self.img3 = None
        #self.img4 = None

        # image list
        self.imgs = []
        actor_list = []

        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(20)

            world = client.get_world()
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            blueprint_library = world.get_blueprint_library()

            bp = blueprint_library.filter('model3')[0]
            print(bp)

            spawn_point = random.choice(world.get_map().get_spawn_points())

            self.vehicle = world.spawn_actor(bp, spawn_point)
            #self.vehicle.apply_control(carla.self.VehicleControl(throttle=1.0, steer=0.0))
            #self.vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

            actor_list.append(self.vehicle)

            # https://carla.readthedocs.io/en/latest/cameras_and_sensors
            # get the blueprint for this sensor
            blueprint = blueprint_library.find('sensor.camera.rgb')
            # change the dimensions of the image
            blueprint.set_attribute('image_size_x', f'{self.IM_WIDTH}')
            blueprint.set_attribute('image_size_y', f'{self.IM_HEIGHT}')
            blueprint.set_attribute('fov', '90')
            blueprint.set_attribute('sensor_tick', '0.1')

            cam_rotation1 = carla.Rotation(0,90,0)
            cam_rotation2 = carla.Rotation(0,180,0)
            cam_rotation3= carla.Rotation(0,270,0)
            cam_location = carla.Location(z=2.0)

            # Adjust sensor relative to self.vehicle
            spawn_point = carla.Transform(cam_location)
            spawn_point_1 = carla.Transform(cam_location,cam_rotation1)
            spawn_point_2 = carla.Transform(cam_location,cam_rotation2)
            spawn_point_3 = carla.Transform(cam_location,cam_rotation3)
            # spawn the sensor and attach to self.vehicle.
            sensor = world.spawn_actor(blueprint, spawn_point, attach_to=self.vehicle)
            sensor1 = world.spawn_actor(blueprint, spawn_point_1, attach_to=self.vehicle)
            sensor2 = world.spawn_actor(blueprint, spawn_point_2, attach_to=self.vehicle)
            sensor3 = world.spawn_actor(blueprint, spawn_point_3, attach_to=self.vehicle)
            # add sensor to list of actors
            actor_list.append(sensor)
            actor_list.append(sensor1)
            actor_list.append(sensor2)
            actor_list.append(sensor3)
            # do something with this sensor

            


            # sensor image will be saved be def save_image
            sensor.listen(lambda data: self.process_img(data,self.pub1))
            sensor1.listen(lambda data: self.process_img(data,self.pub2))
            sensor2.listen(lambda data: self.process_img(data,self.pub3))
            sensor3.listen(lambda data: self.process_img(data,self.pub4))

            while True:
                world.tick()
                

        finally:
            settings.synchronous_mode = False
            world.apply_settings(settings)
            print('destroying actors')
            for actor in actor_list:
                actor.destroy()
            print('done.')

    def process_img(self, image,pub):
        i = np.ndarray(shape=((image.height, image.width, 4)), dtype=np.uint8, buffer=image.raw_data)[:, :, 0:3]
        pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(np.array(i), encoding='passthrough'))

        return i/255.0


if __name__ == '__main__':

    try:
        asd = converter()
    

    except os.error:
        print('fail')
