import rospy
import util.panorama as panorama
from sensor_msgs.msg import Image
import os
import numpy as np
import datetime
import csv
import cv2
import cv_bridge

# subscribe images and make one image
class img2imgs:

    def __init__(self, point = None, f = 570, publishing_mode = False, saving_mode = True):
        
        
        self.images = [None, None, None, None]
        self.point = self.point_loader()
        self.f = f
        self.publishing_mode = publishing_mode
        self.saving_mode = saving_mode
        
        rospy.init_node('subscriber_panorama')

        if publishing_mode == True:
            #rospy.init_node('panorama_image_publisher')
            self.pub = rospy.Publisher("panorama_image_topic", Image)

        self.buffer = 0
        self.count = 1
        self.first_dir = datetime.datetime.now().strftime("%y%m%d_%H%M%S")

        
        # ros type Image
        self.sub1 = rospy.Subscriber('image_topic_1', Image, self.callback1)
        self.sub1 = rospy.Subscriber('image_topic_2', Image, self.callback2)
        self.sub1 = rospy.Subscriber('image_topic_3', Image, self.callback3)
        self.sub1 = rospy.Subscriber('image_topic_4', Image, self.callback4)

        rospy.spin()

    def callback1(self, image_data):
        # ros image to numpy matrix
        self.images[0] = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        #print(self.buffer)
        if self.buffer < 10:
            self.buffer += 1

        #print(self.images[0].shape)
    def callback2(self, image_data):
        self.images[1] = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
    def callback3(self, image_data):
        self.images[2] = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
    def callback4(self, image_data):
        self.images[3] = np.frombuffer(image_data.data, dtype=np.uint8).reshape(image_data.height, image_data.width, -1)
        
        if self.buffer > 3:
            #print('#### Type check ####')
            #print(type(self.images[0]))
            #print(self.images[0].shape)
            #print(len(self.images))
            if self.point == None:
                self.pano_image, self.position = panorama.make_image(self.images, self.point, self.f)
            else:
                self.pano_image = panorama.make_image(self.images, self.point, self.f)
            if self.saving_mode == True:
                self.save_image(self.pano_image, self.first_dir)
            if self.publishing_mode == True:
                self.pub.publish(cv_bridge.CvBridge().cv2_to_imgmsg(np.array(self.pano_image), encoding='passthrough'))

    def save_image(self, image_data, directory):
        if not os.path.isdir(directory):
            os.mkdir(directory)
        path = directory + '/' + str(self.count) + '.jpg'
        cv2.imwrite(path, image_data)
        self.count += 1

    def point_loader(self):
        with open('position.csv','r', encoding='utf-8') as f:
            rdr = csv.reader(f)
            for line in rdr:
                result = line

        for i in range(len(result)):
            result[i] = int(result[i])

        return result


if __name__ == '__main__':
    
    try:
        asd = img2imgs(publishing_mode=True, saving_mode=False)
    except os.error:
        print('fail')