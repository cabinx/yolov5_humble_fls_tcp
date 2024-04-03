# Created by cabin as message transport center between modules(yolo, sonar...) on nx board and the auv
import socket
import struct
import threading
import subprocess
import time
import logging

class center_server():
    def __init__(self):
        self.auv_ip = '192.168.162.75'            # the server ip
        self.auv_port = 8088
        
        self.nx_ip = '192.168.162.75'             # the board ip
        self.data_port = 8080                     # yolo pos info & object image data tansfer port
        self.control_center_port = 8081           # cmd and cmd callback of each module transfer port
        self.sonar_param_port = 8082              # sonar parameters setting cmd tansfer port

        # connect the auv
        self.nx_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.nx_client.connect((self.auv_ip,self.auv_port))

        # yolo server for the yolo module on the nx
        self.yolo_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.yolo_server.bind((self.nx_ip, self.data_port))                          # bind the data port
        self.yolo_server.listen(5)   

        # control_center server for the control_center module on the nx
        self.control_center_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control_center_server.bind((self.nx_ip, self.control_center_port))                          # bind the control_center port
        self.control_center_server.listen(5)

        # sonar parameters parameters setting server for the sonar module on the nx
        self.sonar_param_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sonar_param_server.bind((self.nx_ip, self.sonar_param_port))                          # bind the sonar port
        self.sonar_param_server.listen(5)
        #self.sonar_param_socket, self.sonar_param_addr = self.sonar_param_server.accept()

        # start the control center for the modules such as yolo, sonar, offline test...
        #control_center_cmd = 'cd ~/home/pcl-02/yolov5_humble_fls_tcp/src/TCP_IP && python3 control_center.py'
        #control_center_subprocess = subprocess.Popen(control_center_cmd, shell=True, executable="/bin/bash")

        # info the auv TCP has established
        es_cmd = b'\xaa\xbb\xcc\xdd'
        self.nx_client.send(es_cmd)

    # recv the yolo msg (pos and captured image) and sned to auv
    def recv_yolo_msg(self):
        self.yolo_socket, self.yolo_addr = self.yolo_server.accept()     
        while True:
            yolo_msg = self.yolo_socket.recv(65534)
            if len(yolo_msg) > 0:
                self.nx_client.sendall(yolo_msg)
            if yolo_msg == b'':
                self.yolo_socket, self.yolo_addr = self.yolo_server.accept()

    # send the control_center cmd from auv to the control_center module on nx
    def recv_control_center_msg(self):
        #self.sonar_param_socket, self.sonar_param_addr = self.sonar_param_server.accept()  
        while True:          
            cmd_msg = self.nx_client.recv(1024)
            if len(cmd_msg) > 3:
                # send the control_center cmd to the control_center module
                if cmd_msg[-4:] == b'\xee\xff\xee\xff':
                    self.control_center_socket.send(cmd_msg)
                # send the sonar parameters cmd to the sonar module
                elif cmd_msg[-4:] == b'\xee\xaa\xee\xff':
                    self.sonar_param_socket.send(cmd_msg)
                else:
                    print("cmd from auv error, no such cmd...")

    # recv the callback state msg and send to auv
    def recv_control_center_callback_msg(self):
        self.control_center_socket, self.control_center_addr = self.control_center_server.accept()
        #self.sonar_param_socket, self.sonar_param_addr = self.sonar_param_server.accept()
        while True:
            control_center_callback_msg = self.control_center_socket.recv(1024)
            if len(control_center_callback_msg) > 0:
                self.nx_client.send(control_center_callback_msg)

    # recv sonar img and send to auv
    def rcv_sonar_img_msg(self):
        self.sonar_param_socket, self.sonar_param_addr = self.sonar_param_server.accept()
        while True:
            img_msg = self.sonar_param_socket.recv(65536)
            if img_msg[0:4] == b'\xff\xcc\xff\xcc':
                self.nx_client.sendall(img_msg)
            if img_msg == b'':
                self.sonar_param_socket, self.sonar_param_addr = self.sonar_param_server.accept()

    def keep_tcp_alive(self):
        try:
            alive_msg = b'\xef\xef\xfe\xfe'
            self.nx_client.send(alive_msg)
            time.sleep(60)
        except Exception as e:
            print("sonar alive error: ",e)    

if __name__ == '__main__':
    nx_center = center_server()
    t1 = threading.Thread(target=nx_center.recv_control_center_msg, daemon=True)
    t2 = threading.Thread(target=nx_center.recv_control_center_callback_msg, daemon=True)
    t3 = threading.Thread(target=nx_center.rcv_sonar_img_msg, daemon=True)
    t4 = threading.Thread(target=nx_center.keep_tcp_alive, daemon=True)
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    nx_center.recv_yolo_msg()