# Created by cabin for starting yolo and sonar through TCP, 1 for yolo start, 2 for yolo finish,
# 3 for sonar start, 4 for sonar stop, 5 for captrue image, PengCheng Lab, 2023.03.22
#!/usr/bin/env python3
import socket
import sys
import struct


sonar_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.162.75"
port = 8089
sonar_client.connect((host, port))


cmd =sys.argv[1]
value =sys.argv[2]
arrBuff = ""
if cmd == str(1):
    print("sent the yolo start cmd...")
    arrBuff = bytearray(b'\xee\xaa\xee\xaa\xee\xff\xee\xff')

elif cmd == str(2):
    print("sent the yolo finish cmd...")
    arrBuff = bytearray(b'\xee\xaa\xee\xbb\xee\xff\xee\xff')

elif cmd == str(3):
    print("sent the sonar start cmd...")
    arrBuff = bytearray(b'\xee\xbb\xee\xaa\xee\xff\xee\xff')

elif cmd == str(4):
    print("sent the sonar finish cmd...")
    arrBuff = bytearray(b'\xee\xbb\xee\xbb\xee\xff\xee\xff')

elif cmd == str(5):
    print("sent the offline test start cmd...")
    arrBuff = bytearray(b'\xee\x88\xee\x88\xee\xff\xee\xff')

elif cmd == str(6):
    print("sent the offline test stop cmd...")
    arrBuff = bytearray(b'\xee\x88\xee\x99\xee\xff\xee\xff')

elif cmd == str(7):
    print("sent capture object img cmd...")
    arrBuff = bytearray(b'\xff\xbb\xff\xbb\xee\xff\xee\xff')

elif cmd == str(8):
    print("recording sonar data...")
    arrBuff = bytearray(b'\xee\xbb\xee\xdd\xee\xff\xee\xff')

elif cmd == str(9):
    print("stop record sonar data...")
    arrBuff = bytearray(b'\xee\xbb\xee\xdf\xee\xff\xee\xff')

elif cmd == str(11):
    print("switch the low/high frequency mode...")
    arrBuff = b'\xee\xbb\xee\x01'
    arrBuff += struct.pack('i', int(value))
    arrBuff += b'\xee\xaa\xee\xff'

elif cmd == str(12):
    print("set gamma correction value: " + str(value))
    arrBuff = b'\xee\xbb\xee\x02'
    arrBuff += struct.pack('i', int(value))
    arrBuff += b'\xee\xaa\xee\xff'

elif cmd == str(13):
    print("set sonar range: " + str(value))
    arrBuff = b'\xee\xbb\xee\x03'
    arrBuff += struct.pack('i', int(value))
    arrBuff += b'\xee\xaa\xee\xff'

elif cmd == str(14):
    print("set the gain value: " + str(value))
    arrBuff = b'\xee\xbb\xee\x04'
    arrBuff += struct.pack('f', float(value))
    arrBuff += b'\xee\xaa\xee\xff'

elif cmd == str(15):
    print("set the velocity of sound: " + str(value))
    arrBuff = b'\xee\xbb\xee\x05'
    arrBuff += struct.pack('f', float(value))
    arrBuff += b'\xee\xaa\xee\xff'

elif cmd == str(16):
    print("set the salinity: " + str(value))
    arrBuff = b'\xee\xbb\xee\x06'
    arrBuff += struct.pack('f', float(value))
    arrBuff += b'\xee\xaa\xee\xff'

else:
    print("no such cmd")

if len(arrBuff):
    a = sonar_client.sendall(arrBuff)
    print(a)

sonar_client.close()
