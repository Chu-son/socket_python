import socket
import struct
from math import *
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append("C:/Users/user/Documents/Visual Studio 2013/Projects/ICPtest/ICPtest")
from iterative_closest_point import *

def calc_local_coord(dataList):
    boundary = int(( len(dataList) - 1 ) / 2)
    radian_list = dataList[:boundary]
    distance_list = dataList[boundary:len(dataList) - 1]

    ret_x_list = []
    ret_y_list = []

    for ( rad , dist ) in zip( radian_list , distance_list ):
        ret_x_list.append( dist * cos( rad ) )
        ret_y_list.append( - dist * sin( rad ) )

    return ret_x_list , ret_y_list

def calc_global_coord(dataList):
    data_array = np.array(dataList)
    print(data_array)

    pos = [now - pre for (now,pre) in zip(now_pos,pre_pos)]
    dir = [-radians(now - pre) for (now,pre) in zip(now_dir,pre_dir)]

    R = numpy.array( [[ cos(dir[1]), sin(dir[1])],
                      [-sin(dir[1]), cos(dir[1])]] )
    T = numpy.array( [[ pos[2]] , [pos[0]] ] )

    data_array = R.dot(data_array)
    data_array = np.array([data_array[0] + T[0],
                           data_array[1] + T[1]])
    print(data_array)
    return data_array


#点群データから画像作成
def plotPoint2Image(data,img):
    imgMap = img.load()
    coefficient = 100.0 / 5.0
    origin_x = img.size[0] / 2.0
    origin_y = img.size[1] / 2.0
    
    for (raw_x, raw_y) in zip(data[0],data[1]):
        if raw_x == 0 and raw_y == 0:continue
        x = int(raw_x * coefficient + origin_x)
        y = int(raw_y * coefficient + origin_y)
        
        if imgMap[x,y] != 250:
           imgMap[x,y] += 50
    return img

serverAddr = "localhost"
serverPort = 8000

client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_sock.connect((serverAddr, serverPort))

img = Image.new("L",(400,400))
icp = MyIterativeClosestPoint()

clientInput = ''
pre_pos = []
pre_dir = []
now_pos = []
now_dir = []

while True:
    clientInput= input(' please input > ')
    if clientInput == "exit":
        break
    clientInput += ' \n'
    client_sock.send(clientInput.encode('utf-8'))
    print("send")

    response = ()
    while True:
        receivedata = client_sock.recv(1024)
        fm = "f" * int( len(receivedata) / struct.calcsize("f"))
        tmp = struct.unpack( fm, receivedata );
        response += tmp
        if -1.0 in tmp:
            break

    if "lrf" in clientInput:
        #img = plotPoint2Image( icp.ICPMatching( calc_local_coord(response)) , img)
        img = plotPoint2Image( calc_global_coord( calc_local_coord(response)) , img)

        #画像をarrayに変換
        im_list = np.asarray(img)
        #貼り付け
        plt.imshow(im_list)
        plt.gray()
        #表示
        plt.pause(.01)

        #img.show()
        #print (response)

    elif "move" in clientInput and "get" in clientInput:
        pre_pos = now_pos
        pre_dir = now_dir
        now_pos = response[0:3]
        now_dir = response[3:6]

client_sock.close()