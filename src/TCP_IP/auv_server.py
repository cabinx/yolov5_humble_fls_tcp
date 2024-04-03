# Created by cabin for receiving the Nvidia nx board massage and sending cmd through TCP, printing the boject class, probability, azimuth 
# and range, displaying the object image
import socket
import struct
import cv2
import numpy as np
import threading
import time

class object:
    def __init__(self):
        self.object_class = ""
        self.object_probability = ""
        self.object_azimuth = ""
        self.object_range = ""

class objects:
    def __init__(self):
        self.object = []

class object_image:
    def __init__(self):
        self.object_class = ""
        self.objcect_probability =""
        self.object_azimuth = ""
        self.object_range = ""

class objects_image:
    def __init__(self):
        self.object_info = []
        self.object_image_data = []

class auv_server():
    def __init__(self):
        self.recv_images = objects_image()
        self.object_count = 0
        self.display_image_flag = False
        self.display_sonar_flag = False
        self.sonar_image_data = b''

        self.auv_ip = '192.168.162.75'
        self.auv_port = 8088

        self.cmd_test_port = 8089
        # tcp server setting
        self.auv_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.auv_server.bind((self.auv_ip, self.auv_port))
        self.auv_server.listen(5)
        self.conn, self.addr = self.auv_server.accept()

        self.test_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.test_server.bind((self.auv_ip, self.cmd_test_port))
        self.test_server.listen(5)

        self.cmd_callback_state = {
            b'\xee\xaa\xee\xaa' : "start yolo... ",
            b'\xee\xaa\xee\xa9' : "yolo is running...",
            b'\xee\xaa\xee\xbb' : "yolo closed... ",
            b'\xee\xaa\xee\xb9' : "no yolo is running",
            b'\xee\xbb\xee\xaa' : "start sonar... ",
            b'\xee\xbb\xee\xa9' : "sonar is running...",
            b'\xee\xbb\xee\xbb' : "sonar closed... ",
            b'\xee\xbb\xee\xb9' : "no sonar is running",
            b'\xee\x88\xee\x88' : "start offline test... ",
            b'\xee\x88\xee\xa9' : "offline test is running...",
            b'\xee\x88\xee\x99' : "finish offline test .. ",
            b'\xee\x88\xee\xb9' : "offline test is running... "
        }

    # receive the data from auv
    def recv_msg(self):
        while True:
            recv_msg = self.conn.recv(65534)
            pkg_head = recv_msg[0:4]

            # check the tcp connection to the nx state
            if pkg_head == b'\xaa\xbb\xcc\xdd':
                time.sleep(2)
                print("connection between auv_server and centerserver established")

            # the objects positions info
            if pkg_head == b'\xff\xaa\xff\xaa':
                object_num = struct.unpack('i',recv_msg[4:8])[0]
                recv_objects = objects()
                for i in range(object_num):
                    recv_object = object()
                    recv_object.object_class = struct.unpack('i',recv_msg[8 + i * 16 : 12 + i * 16])[0]
                    recv_object.object_probability = struct.unpack('f',recv_msg[12 + i * 16 : 16 + i * 16])[0]
                    recv_object.object_azimuth = struct.unpack('f',recv_msg[16 + i * 16 : 20 + i * 16])[0]
                    recv_object.object_range = struct.unpack('f',recv_msg[20 + i * 16 : 24 + i * 16])[0]
                    recv_objects.object.append(recv_object)

                pkg_end = recv_msg[24 + 16 * (object_num - 1) : 28 + 16 * (object_num - 1)]
                # check package info and print the object info
                if pkg_end == b'\xff\xa0\xff\xa0':
                    if len(recv_objects.object):
                            cur_num = 0
                            print("")
                            while cur_num < len(recv_objects.object):
                                print("object class: " + str(recv_objects.object[cur_num].object_class) + "; object probability: " + str(recv_objects.object[cur_num].object_probability) + \
                                    "; object azimuth: " + str(recv_objects.object[cur_num].object_azimuth) + "; object range: " + str(recv_objects.object[cur_num].object_range))
                                cur_num += 1
                            print("")
                    else:
                        #a = 1
                        print("no object found...")
                else:
                    print("receive error package ending!!!")

            # object images
            if pkg_head == b'\xff\xbb\xff\xbb':
                # object num in one image capture
                object_num = struct.unpack('i',recv_msg[4:8])[0]

                # current object info in the objects of the captured image
                object_info = object()
                cur_object_num = struct.unpack('i',recv_msg[8:12])[0]
                object_info.object_class = struct.unpack('i', recv_msg[12:16])[0]
                object_info.object_probability = struct.unpack('f', recv_msg[16:20])[0]
                object_info.object_azimuth = struct.unpack('f', recv_msg[20:24])[0]
                object_info.object_range = struct.unpack('f', recv_msg[24:28])[0]
                self.recv_images.object_info.append(object_info)
                # current object image info
                img_height = struct.unpack('i',recv_msg[28:32])[0]
                img_width = struct.unpack('i',recv_msg[32:36])[0]
                img_channel = struct.unpack('i',recv_msg[36:40])[0]
                img_data_len = struct.unpack('i',recv_msg[40:44])[0]

                img_data = recv_msg[44:44 + img_data_len]
                pkg_end = recv_msg[44 + img_data_len : 48 + img_data_len]
                #print(pkg_end)

                if pkg_end == b'\xff\xb0\xff\xb0':
                    self.recv_images.object_image_data.append(img_data)

                if self.object_count < object_num:
                    self.object_count += 1
                
                if self.object_count == object_num:
                    self.display_image_flag = True
                    self.object_count = 0

            # sonar image
            if pkg_head == b'\xff\xcc\xff\xcc':
                #img_range = struct.unpack('f', recv_msg[4:8])[0]
                #img_azimuth = struct.unpack('f', recv_msg[8:12])[0]
                #img_height = struct.unpack('i', recv_msg[12:16])[0]
                #img_width = struct.unpack('i', recv_msg[16:20])[0]
                #img_len = struct.unpack('i', recv_msg[20:24])[0]

                #self.sonar_image_data = recv_msg[24:24 + img_data_len]
                #self.display_sonar_flag = True

                #print(img_range, img_azimuth, img_height, img_width, img_len)
                print("discard...")
            
            # cmd callback state msg
            if pkg_head == b'\xee\xff\xee\xff':
                print(self.cmd_callback_state[recv_msg[4:8]])

    # display 
    def display_objects_image(self):
        #print("image show start")
        while True:
            #if len(self.recv_images.object_image_data):
            if self.display_image_flag:
                self.display_image_flag = False
                cout1 = 0
                while cout1 < len(self.recv_images.object_image_data):
                    # img data decode
                    image = cv2.imdecode(np.array(bytearray(self.recv_images.object_image_data[cout1]), dtype='uint8'), cv2.IMREAD_UNCHANGED)

                    # the object class, probability, azimuth and range as the image title
                    str_class = "class: " + str(self.recv_images.object_info[cout1].object_class)
                    str_probability = "pro: " + "%.2f" % self.recv_images.object_info[cout1].object_probability
                    str_pos = "pos: (" + "%.2f" % self.recv_images.object_info[cout1].object_azimuth + ", " + "%.2f" % self.recv_images.object_info[cout1].object_range + "m)"
                    image_title = str_class + " " + str_probability + " " + str_pos

                    cv2.imshow(image_title, image)
                    cout1 += 1
                    if cout1 == len(self.recv_images.object_image_data):
                        cv2.waitKey(0)
                        cv2.destroyAllWindows()
                        self.recv_images.object_info.clear()
                        self.recv_images.object_image_data.clear()


    def test_cmd(self):
        while True:
            self.test_conn, self.test_addr = self.test_server.accept()
            test_cmd_msg = self.test_conn.recv(1024)
            if len(test_cmd_msg)>0:
                self.conn.send(test_cmd_msg)

if __name__ == '__main__':
    auv_server_pcl = auv_server()
    t1 = threading.Thread(target=auv_server_pcl.display_objects_image, daemon=True)
    t2 = threading.Thread(target=auv_server_pcl.test_cmd, daemon=True)
    t1.start()
    t2.start()
    auv_server_pcl.recv_msg()