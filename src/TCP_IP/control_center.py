# Created by cabin as cmd module on nx board for controlling yolo and sonar, PengCheng Lab, 2023.04.07
#!/usr/bin/env python3
import socket
import os
import subprocess
import threading
import time
import shlex
import psutil

class control_center:
    def __init__(self):
        self.yolo_subprocess = None
        self.yolo_running = False
        self.offline_test_subprocess = None
        self.sonar_subprocess = None
        self.capture_img_subprocess = None
        self.sonar_running = False
        self.offline_test_running = False
        self.record_state = False
        self.record_subprocess = None
        self.record_cmd = None

        self.cmd_callback_header = b'\xee\xff\xee\xff'     # cmd callback tcp package header to the AUV

        # yolo cmd
        self.start_yolo_msg = b'\xee\xaa\xee\xaa\xee\xaa\xee\xff'          # start yolo cmd, and info the AUV yolo sucessfully started
        self.finish_yolo_msg = b'\xee\xaa\xee\xbb\xee\xaa\xee\xff'         # finish yolo cmd, and info the AUV yolo sucessfully finished
        self.yolo_is_running = b'\xee\xaa\xee\xa9\xee\xaa\xee\xff'         # info the AUV yolo is running, could not start yolo now
        self.yolo_is_no_running = b'\xee\xaa\xee\xb9\xee\xaa\xee\xff'      # info the AUV no yolo is running, could not kill yolo now

        # sonar cmd
        self.start_sonar_msg = b'\xee\xbb\xee\xaa\xee\xbb\xee\xff'         # start sonar cmd, and info the AUV sonar sucessfully started
        self.finish_sonar_msg = b'\xee\xbb\xee\xbb\xee\xbb\xee\xff'        # stop sonar cmd, and info the AUV sonar sucessfully stopped
        self.sonar_is_running = b'\xee\xbb\xee\xa9\xee\xbb\xee\xff'        # info the AUV sonar is running, could not start sonar now
        self.sonar_is_no_running = b'\xee\xbb\xee\xb9\xee\xbb\xee\xff'     # info the AUV sonar is running, could not stop sonar now
        
        # offline test cmd
        self.start_offline_test_msg = b'\xee\x88\xee\x88\xee\x88\xee\xff'         # start offline_test cmd, and info the AUV offline_test sucessfully started       
        self.finish_offline_test_msg = b'\xee\x88\xee\x99\xee\x88\xee\xff'        # stop offline_test cmd, and info the AUV offline_test sucessfully stopped
        self.offline_test_is_running = b'\xee\x88\xee\xa9\xee\x88\xee\xff'        # info the AUV offline_test is running, could not start offline_test now
        self.offline_test_is_no_running = b'\xee\x88\xee\xb9\xee\x88\xee\xff'     # info the AUV no offline_test is running, could not stop offline_test now
        
        # capture object image cmd
        self.capture_object_img_msg = b'\xff\xbb\xff\xbb'

        # record bag cmd
        self.start_rec_bag_msg = b'\xee\xbb\xee\xdd'
        self.finish_rec_bag_msg = b'\xee\xbb\xee\xdf'

        self.start_yolo_cmd = 'cd ~/yolov5_humble_fls_tcp && source install/setup.bash && ros2 launch yolov5_humble_fls start.launch.py'
        self.finish_yolo_cmd = 'ps -ef | grep yolov5_humble_fls| grep -v grep| awk \'{print$2}\'| xargs kill -9'
        self.start_sonar_cmd = 'cd ~/oculus_ros2 && source install/setup.bash && ros2 launch oculus_ros2 m750d.launch.py'
        self.finish_sonar_cmd = 'ps -ef | grep oculus_ros2| grep -v grep| awk \'{print$2}\'| xargs kill -9'
        self.start_offline_test_cmd = 'source /opt/ros/humble/setup.bash && ros2 bag play ~/yolov5_humble_fls_tcp/bag/sonar_mono'
        self.finish_offline_test_cmd = 'ps -ef | grep ros2[[:space:]]bag[[:space:]]play| grep -v grep| awk \'{print$2}\'| xargs kill -9'
        self.capture_object_img_cmd = 'ros2 topic pub --once /yolov5/capture_object_image std_msgs/msg/Bool data:\ true'

        self.sonar_image_topic = "/sonar_image_raw"

        self.nx_ip = '192.168.162.75'
        self.cmd_port = 8081

        self.cmd_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                           # create TCP/IP socket
        self.cmd_client.connect((self.nx_ip, self.cmd_port))

    # send cmd callback state msg to center server
    def send_cmd(self, cmd):
        try:
            cmd_array = self.cmd_callback_header
            cmd_array += cmd
            self.cmd_client.sendall(cmd_array)
        except Exception as e:
            print("callback yolo start cmd error:", e)

    def rcev_cmd(self):
        while True:
            recvmsg = self.cmd_client.recv(1024)

            # start yolov5_ros ros2 launch
            if recvmsg[0:4] == self.start_yolo_msg[0:4]:
                # yolo is running, continue
                if self.yolo_running:
                    print("yolo is running...")
                    # callback the cmd
                    self.send_cmd(self.yolo_is_running)
                # no yolo is running, start yolo
                else:
                    print("start yolo...")
                    self.yolo_subprocess = subprocess.Popen(self.start_yolo_cmd, shell=True, executable="/bin/bash")
                    self.yolo_running = True
                    self.send_cmd(self.start_yolo_msg)
            
            # kill yolov5_ros ros2 launch
            elif recvmsg[0:4] == self.finish_yolo_msg[0:4]:
                # yolo is running, finish yolo
                if self.yolo_running :
                    try:
                        self.yolo_subprocess.kill()
                    except Exception as e:
                        print("first start yolo")
                    os.system(self.finish_yolo_cmd)
                    self.yolo_running = False
                    # callback the cmd
                    self.send_cmd(self.finish_yolo_msg)
                # no yolo is running, continue
                else :
                    print("no yolo is running...")
                    # callback the cmd
                    self.send_cmd(self.yolo_is_no_running)
                    #self.send_callback_cmd_msg(self.yolo_is_no_running)

            # start sonar
            elif recvmsg[0:4] == self.start_sonar_msg[0:4]:
                # sonar is running, continue
                if self.sonar_running:
                    print("sonar is running...")
                    # callback the cmd
                    self.send_cmd(self.sonar_is_running)
                # no sonar is running, start sonar
                else:
                    print("start sonar...")
                    self.sonar_subprocess = subprocess.Popen(self.start_sonar_cmd, shell=True, executable="/bin/bash")
                    self.sonar_running = True
                    self.send_cmd(self.start_sonar_msg)

            # finish sonar
            elif recvmsg[0:4] == self.finish_sonar_msg[0:4]:
                # sonar is running, stop sonar
                if self.sonar_running:
                    try:
                        self.sonar_subprocess.kill()
                    except Exception as e:
                        print("first start sonar")
                    os.system(self.finish_sonar_cmd)
                    self.sonar_running = False
                    #callback the cmd
                    self.send_cmd(self.finish_sonar_msg)
                # no sonar is running, continue
                else:
                    print("no sonar is runnning...")
                    #callback the cmd
                    self.send_cmd(self.sonar_is_no_running)

            
            # start offline test
            elif recvmsg[0:4] == self.start_offline_test_msg[0:4]:
                # offline_test is running, continue
                if self.offline_test_running :
                    print("offline_test is running...")
                    # callback the cmd
                    self.send_cmd(self.offline_test_is_running)
                # no offline_test is running, start offline_test
                else:
                    print("start offline_test...")
                    self.offline_test_subprocess = subprocess.Popen(self.start_offline_test_cmd, shell=True, executable="/bin/bash")
                    self.offline_test_running = True
                    # callback the cmd
                    self.send_cmd(self.start_offline_test_msg)
            
            # stop offline_test
            elif recvmsg[0:4] == self.finish_offline_test_msg[0:4]:
                # offline_test is running, stop offline_test
                if self.offline_test_running :
                    self.offline_test_subprocess.kill()
                    os.system(self.finish_offline_test_cmd)
                    self.offline_test_running = False
                    # callback the cmd
                    self.send_cmd(self.finish_offline_test_msg)
                # no offline_test is running, continue
                else:
                    print("no offline_test is running...")
                    # callback the cmd
                    self.send_cmd(self.offline_test_is_no_running)
    
            # capture the object image and sent back to auv
            elif recvmsg[0:4] == self.capture_object_img_msg[0:4]:
                if self.capture_img_subprocess:
                    self.capture_img_subprocess.kill()
                self.capture_img_subprocess = subprocess.Popen(self.capture_object_img_cmd, shell=True, executable="/bin/bash")

            # start record data...
            elif recvmsg[0:4] == self.start_rec_bag_msg[0:4]:
                if not self.record_state:
                    self.record_state = True
                    cur_time = time.time()
                    cur_time = str(cur_time).replace('.','_')
                    self.record_cmd = 'cd ~/yolov5_humble_fls_tcp/record_data && source ~/yolov5_humble_fls_tcp/install/setup.bash && ros2 bag record -o ' + cur_time + ' ' + self.sonar_image_topic
                    self.record_subprocess = subprocess.Popen(self.record_cmd, shell=True, executable="/bin/bash")
                else:
                    print("bag data is recording...")

            # finish record data...
            elif recvmsg[0:4] == self.finish_rec_bag_msg[0:4]:
                if self.record_state:
                    self.record_cmd = shlex.split(self.record_cmd)
                    record_index = self.record_cmd.index('record')
                    for proc in psutil.process_iter():
                        if "record" in proc.name() and set(self.record_cmd[record_index+1:]).issubset(proc.cmdline()):
                            proc.send_signal(subprocess.signal.SIGINT)
                    self.record_subprocess.send_signal(subprocess.signal.SIGINT)
           
            # error cmd message
            else:
                print("no such cmd: ", recvmsg)

    def keep_tcp_alive(self):
        try:
            alive_msg = b'\xef\xef\xfe\xfe'
            self.cmd_client.send(alive_msg)
            time.sleep(60)
        except Exception as e:
            print("sonar alive error: ",e)
                
if __name__ == '__main__':
    ctrl_center = control_center()
    t1 = threading.Thread(target=ctrl_center.keep_tcp_alive, daemon=True)
    t1.start()
    ctrl_center.rcev_cmd()
