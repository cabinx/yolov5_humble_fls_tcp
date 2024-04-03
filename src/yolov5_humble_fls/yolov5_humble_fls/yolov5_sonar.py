#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
#conda_base_addr = '/home/pcl-02/anaconda3'    # your conda address
#conda_env = 'py310'                          # your conda environment
#python_env = 'python3.10'                      # python environment in your conda environment 
#sys.path.append(conda_base_addr + '/envs/' + conda_env + '/lib/' + python_env + '/site-packages/')

import cv2
import torch
import rclpy
#from rclpy.node import Node
from rclpy.lifecycle import Node
import numpy as np
from sensor_msgs.msg import Image
from object_msgs.msg import ObjectAzimuthRange
from object_msgs.msg import ObjectsAzimuthRange
from std_msgs.msg import Bool

import sys

import socket
import struct
import yaml


class Yolo_Dect(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.declare_parameter('yolov5_path', '')
        yolov5_path = self.get_parameter('yolov5_path').get_parameter_value().string_value
        self.declare_parameter('weight_path', '')
        weight_path = self.get_parameter('weight_path').get_parameter_value().string_value
        self.declare_parameter('image_topic','/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.declare_parameter('objects_info_pub_topic','/objects_info')
        objects_info_pub_topic = self.get_parameter('objects_info_pub_topic').get_parameter_value().string_value
        self.declare_parameter('conf',0.5)
        conf = self.get_parameter('conf').value
        self.declare_parameter('display_image', True)
        self.display_image = self.get_parameter('display_image').get_parameter_value().bool_value

        # tcp parameters
        self.declare_parameter('server_ip', '192.168.162.75')
        self.server_host = self.get_parameter('server_ip').get_parameter_value().string_value
        self.declare_parameter('server_port', 8080)
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        # tcp transport object image min resulotion, here example is 200*200
        self.declare_parameter('image_min_length', 200)
        self.image_min_length = self.get_parameter('image_min_length').get_parameter_value().integer_value
        # tcp transport object image compress quality (0~100), the lower quality, the smaler data size
        self.declare_parameter('image_quality', 10)
        self.image_quality = self.get_parameter('image_quality').get_parameter_value().integer_value

        self.declare_parameter('object_names_file', ' ')
        self.object_names_file = self.get_parameter('object_names_file').get_parameter_value().string_value

        # establish tcp link
        try:
            self.yolo_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.yolo_client.connect((self.server_host, self.server_port))
        except Exception as e:
            tcp_error = "connection error... " + str(self.server_host) + " : " + str(self.server_port)
            print("\033[31;1m %s %s\033[0m" %(tcp_error,e))

        try:
            object_name_file = open(self.object_names_file, 'r')
            print(object_name_file)
            self.object_name = yaml.load(object_name_file.read(), Loader=yaml.FullLoader)
            object_name_file.close()
        except Exception as e:
            print("\033[31;1m %s \033[0m" %e)

        # load local repository(YoloV5:v7.0)
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_path, source='local')       ######
        # which device will be used
        self.declare_parameter('use_cpu', False)
        use_cpu = self.get_parameter('use_cpu').get_parameter_value().bool_value
        if use_cpu:
            self.model.cpu()    ######                                                                       
            self.get_logger().info("use_cpu")
        else:
            self.model.cuda()   ######
            self.get_logger().info("use_cuda")
        #self.model.conf = float(conf)  ######
        self.model.conf = conf  ######
        self.color_image = Image()
        self.getImageStatus = False

        # Load class color
        self.classes_colors = {}

        self.image_capture = None                           # image data for sending
        self.object_image_capture_flag = False              # flag for sending object image data, sending to control board if True
        self.capture_image_h = 0
        self.capture_image_w = 0
        self.capture_image_channel = 0

        # image subscribe
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 1)
        self.object_image_capture_sub = self.create_subscription(Bool, "/yolov5/capture_object_image", self.object_image_capture_callback, 1)
        # output publishers
        self.objects_info_pub = self.create_publisher(ObjectsAzimuthRange, objects_info_pub_topic, 1)

        timer = Node.create_timer(self, 2, self.timer_callback)

    def timer_callback(self):
        if(not self.getImageStatus):
            self.get_logger().info("waiting for image.")

    def image_callback(self, image):
        self.getImageStatus = True
        self.objects_azimuth_range = ObjectsAzimuthRange()
        self.objects_azimuth_range.header = image.header
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.model(self.color_image)                   ######
        # xmin    ymin    xmax    ymax    confidence    class    name

        boxs = results.pandas().xyxy[0].values

        # loading the range and azimuth of sonar
        sonar_info = image.header.frame_id
        mid_index = sonar_info.find(' ')
        sonar_azimuth = float(sonar_info[1 : mid_index])
        sonar_range = float(sonar_info[mid_index + 1 : -1])

        # capture the image to transport
        if self.object_image_capture_flag:
            self.image_capture = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width, -1)
            self.capture_image_h = image.height
            self.capture_image_w = image.width
            self.capture_image_channel = len(image.data) / (image.height * image.width)

        self.dectshow(self.color_image, boxs, sonar_azimuth, sonar_range)
        cv2.waitKey(3)

    def dectshow(self, org_img, boxs, sonar_azimuth, sonar_range):
        img = org_img.copy()

        count = 0
        for i in boxs:
            count += 1

        image_height = img.shape[0]
        image_width = img.shape[1]

        # package header for the azimuth and range tcp package 
        pkg_buff = bytearray(b'\xff\xaa\xff\xaa')
        # objects number the azimuth and range tcp package 
        pkg_buff += bytearray(count.to_bytes(4, byteorder='little'))
        
        for box in boxs:
            # calculate the azimuth and range info and package the info for ros
            object_azimuth_range = ObjectAzimuthRange()
            object_azimuth_range.class_name = box[-1]
            object_azimuth_range.probability = np.float64(box[4])
            object_x = (int(box[0]) + int(box[2])) / 2.0                          #width
            object_y = (int(box[1]) + int(box[3])) / 2.0                          #height
            object_azimuth_range.object_azimuth = (object_x / (image_width / 2.0) - 1.0) * (sonar_azimuth / 2.0)
            object_azimuth_range.object_range = (1.0 - object_y / image_height) * (sonar_range)
            object_azimuth_range.xmin = int(box[0])
            object_azimuth_range.ymin = int(box[1])
            object_azimuth_range.xmax = int(box[2])
            object_azimuth_range.ymax = int(box[3])
            object_azimuth_range.num = int(count)
            self.objects_azimuth_range.object_azimuth_range.append(object_azimuth_range)
            
            # pacakage the azimuth and ranges info for tcp
            object_class = self.object_name[object_azimuth_range.class_name]
            pkg_buff += bytearray(object_class.to_bytes(4, byteorder='little'))
            pkg_buff += struct.pack('<f', object_azimuth_range.probability)
            pkg_buff += struct.pack('<f', object_azimuth_range.object_azimuth)
            pkg_buff += struct.pack('<f', object_azimuth_range.object_range)

            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0,183,3)
                self.classes_colors[box[-1]] = color

            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (int(color[0]), int(color[1]), int(color[2])), 2)

            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10

            object_pro_str = object_azimuth_str = '%.2f' % object_azimuth_range.probability
            object_azimuth_str = '%.2f' % object_azimuth_range.object_azimuth
            object_range_str = '%.2f' % object_azimuth_range.object_range
            object_pose_str = '(' + object_azimuth_str + ', ' + object_range_str + ')'
            text = box[-1] + '-' + object_pro_str + ': ' + object_pose_str
            cv2.putText(img, text, (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        pkg_buff += bytearray(b'\xff\xa0\xff\xa0')
        #self.yolo_client.sendall(pkg_buff)

        self.objects_info_pub.publish(self.objects_azimuth_range)

        # tcp transport the cut and compressed object images data
        # ensure get the image
        if self.capture_image_h * self.capture_image_w > 0:
            object_image_num = len(self.objects_azimuth_range.object_azimuth_range)
            # ensure object in the image
            if object_image_num > 0:
            #if object_image_num > 1:   #ensure several object for test
                try:                    
                    object_image_count = 0
                    for object_a_r in self.objects_azimuth_range.object_azimuth_range:
                        # cut the object in the image
                        if object_a_r.xmax - object_a_r.xmin < (self.image_min_length / 2):
                            cut_xmin = int((object_a_r.xmin + object_a_r.xmax) / 2.0 - self.image_min_length / 2)
                            cut_xmax = int((object_a_r.xmin + object_a_r.xmax) / 2.0 + self.image_min_length / 2)
                        else:
                            cut_xmin = int(object_a_r.xmin - 50)
                            cut_xmax = int(object_a_r.xmax + 50)

                        if object_a_r.ymax - object_a_r.ymin < (self.image_min_length / 2):
                            cut_ymin = int((object_a_r.ymin + object_a_r.ymax) / 2.0 - self.image_min_length / 2)
                            cut_ymax = int((object_a_r.ymin + object_a_r.ymax) / 2.0 + self.image_min_length / 2)
                        else:
                            cut_ymin = int(object_a_r.ymin - 50)
                            cut_ymax = int(object_a_r.ymax + 50)
                        cut_image = self.image_capture[max(0, cut_ymin) : min(self.image_capture.shape[0], cut_ymax), max(0, cut_xmin) : min(self.image_capture.shape[1], cut_xmax)]
                            
                        # get the class, probability, azimuth and range of the object
                        img_buff = bytearray(b'\xff\xbb\xff\xbb')
                        img_buff += bytearray(object_image_num.to_bytes(4, byteorder='little'))
                        img_buff += bytearray(object_image_count.to_bytes(4, byteorder='little'))
                        object_class = self.object_name[object_a_r.class_name]
                        img_buff += bytearray(object_class.to_bytes(4, byteorder='little')) 
                        img_buff += struct.pack('<f', object_a_r.probability) 
                        img_buff += struct.pack('<f', object_a_r.object_azimuth)
                        img_buff += struct.pack('<f', object_a_r.object_range)

                        # get the height, width and channel of the object image
                        cut_img_height = cut_image.shape[0]
                        cut_img_width = cut_image.shape[1]
                        cut_img_channel = cut_image.shape[2]
                        img_buff += bytearray(cut_img_height.to_bytes(4, byteorder='little')) 
                        img_buff += bytearray(cut_img_width.to_bytes(4, byteorder='little')) 
                        img_buff += bytearray(cut_img_channel.to_bytes(4, byteorder='little'))                            
                            
                        # encode the object image
                        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
                        result, encode_image = cv2.imencode('.jpg', cut_image, encode_param)
                        # get object image data length
                        cut_image_data_len = len(encode_image)
                        img_buff += bytearray(cut_image_data_len.to_bytes(4, byteorder='little'))
                        # objcet image data
                        img_buff += bytearray(encode_image)

                        # print("the trans image length is", len(encode_image))
                        
                        img_buff += bytearray(b'\xff\xb0\xff\xb0')
                        self.yolo_client.sendall(img_buff)

                        # print("sent image " + str(object_image_count))
                        object_image_count += 1
                        
                    self.capture_image_h = 0
                    self.capture_image_w = 0
                    self.image_capture = None
                    self.object_image_capture_flag = False

                except Exception as e:
                    self.capture_image_h = 0
                    self.capture_image_w = 0
                    self.image_capture = None
                    print("image transport error... ", e)

        if self.display_image:
            cv2.imshow('YOLOv5', img)

    def object_image_capture_callback(self, msg):
        if msg.data:
            self.object_image_capture_flag = True

def main(args=None):
    rclpy.init(args=args)
    yolo_dect_node = Yolo_Dect()
    rclpy.spin(yolo_dect_node)
    yolo_dect_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
