import socket
import struct
from math import *
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np
import sys
#sys.path.append("C:/Users/user/Documents/Visual Studio 2013/Projects/ICPtest/ICPtest")
sys.path.append("C:/Users/KOSUKE/Source/Repos/ICPsample_python/ICPtest")
from iterative_closest_point import *

def calc_local_coord(dataList):
    boundary = int( len(dataList) / 2)
    radian_list = dataList[:boundary]
    distance_list = dataList[boundary:len(dataList)]

    ret_x_list = []
    ret_y_list = []

    for ( rad , dist ) in zip( radian_list , distance_list ):
        ret_x_list.append( dist * cos( rad ) )
        ret_y_list.append( - dist * sin( rad ) )

    return ret_x_list , ret_y_list

def calc_global_coord(dataList):
    data_array = np.array(dataList)
    print(data_array)

    pos = [now - init for (now,init) in zip(now_pos,init_pos)]
    dir = [-radians(now - init) for (now,init) in zip(now_dir,init_dir)]

    R = numpy.array( [[ cos(dir[1]), sin(dir[1])],
                      [-sin(dir[1]), cos(dir[1])]] )
    T = numpy.array( [[ pos[0]] , [-pos[2]] ] )

    data_array = np.array([data_array[0] + 0.6,
                           data_array[1]])
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

def get_data_list(sock, command ):
    command += ' \n'
    sock.send(command.encode('utf-8'))
    print("send")

    response = ()
    while True:
        receivedata = sock.recv(1024)
        fm = "f" * int( len(receivedata) / struct.calcsize("f"))
        tmp = struct.unpack( fm, receivedata );
        response += tmp
        if -1.0 in tmp:
            return response[0:-1]


def get_movement(sock):
    response = get_data_list(sock , "move get")
    if init_pos == [] and init_dir == []:
        init_pos = response[0:3]
        init_dir = response[3:6]
    now_pos = response[0:3]
    now_dir = response[3:6]

def get_lrf_data( sock ):
    response = get_data_list(sock , "lrf a")
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

class CommMyRobotSimulator:
    def __init__(self, serverPort = 8000, serverAddr = "localhost"):
        self.serverAddr = serverAddr
        self.serverPort = serverPort

        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.connect((serverAddr, serverPort))

        self.img = Image.new("L",(400,400))
        self.icp = MyIterativeClosestPoint()

        self.init_pos = []
        self.init_dir = []
        self.now_pos = []
        self.now_dir = []

    def calc_local_coord(self, dataList):
        boundary = int( len(dataList) / 2)
        radian_list = dataList[:boundary]
        distance_list = dataList[boundary:len(dataList)]

        ret_x_list = []
        ret_y_list = []

        for ( rad , dist ) in zip( radian_list , distance_list ):
            ret_x_list.append( dist * cos( rad ) )
            ret_y_list.append( - dist * sin( rad ) )

        return ret_x_list , ret_y_list

    def calc_global_coord(self, dataList):
        data_array = np.array(dataList)

        pos = [now - init for (now,init) in zip(self.now_pos,self.init_pos)]
        dir = [-radians(now - init) for (now,init) in zip(self.now_dir,self.init_dir)]

        R = numpy.array( [[ cos(dir[1]), sin(dir[1])],
                          [-sin(dir[1]), cos(dir[1])]] )
        T = numpy.array( [[ pos[0]] , [-pos[2]] ] )

        data_array = np.array([data_array[0] + 0.6,
                               data_array[1]])
        data_array = R.dot(data_array)
        data_array = np.array([data_array[0] + T[0],
                               data_array[1] + T[1]])

        return data_array


    #点群データから画像作成
    def plotPoint2Image(self, data):
        imgMap = self.img.load()
        coefficient = 100.0 / 5.0
        origin_x = self.img.size[0] / 2.0
        origin_y = self.img.size[1] / 2.0
    
        for (raw_x, raw_y) in zip(data[0],data[1]):
            if raw_x == 0 and raw_y == 0:continue
            x = int(raw_x * coefficient + origin_x)
            y = int(raw_y * coefficient + origin_y)
        
            if imgMap[x,y] != 250:
               imgMap[x,y] += 50
        return self.img

    def get_data_list(self, command ):
        command += ' \n'
        self.client_sock.send(command.encode('utf-8'))
        print("send")

        response = ()
        while True:
            receivedata = self.client_sock.recv(1024)
            fm = "f" * int( len(receivedata) / struct.calcsize("f"))
            tmp = struct.unpack( fm, receivedata );
            response += tmp
            if -1.0 in tmp:
                return response[0:-1]


    def get_movement(self):
        response = self.get_data_list( "move get")
        if self.init_pos == [] and self.init_dir == []:
            self.init_pos = response[0:3]
            self.init_dir = response[3:6]
        self.now_pos = response[0:3]
        self.now_dir = response[3:6]

    def get_lrf_data(self):
        response = self.get_data_list("lrf a")
        #img = plotPoint2Image( icp.ICPMatching( calc_local_coord(response)) , img)
        self.img = self.plotPoint2Image( self.calc_global_coord( self.calc_local_coord(response)))

        #画像をarrayに変換
        im_list = np.asarray(self.img)
        #貼り付け
        plt.imshow(im_list)
        plt.gray()
        #表示
        plt.pause(.01)

        #img.show()
        #print (response)

    def make_map_loop(self):
        while True:
            self.get_movement()
            self.get_lrf_data()


comm = CommMyRobotSimulator()
comm.make_map_loop()

serverAddr = "localhost"
serverPort = 8000

client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_sock.connect((serverAddr, serverPort))

img = Image.new("L",(400,400))
icp = MyIterativeClosestPoint()

clientInput = ''
init_pos = []
init_dir = []
now_pos = []
now_dir = []

while False:
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
        if init_pos == [] and init_dir == []:
            init_pos = response[0:3]
            init_dir = response[3:6]
        now_pos = response[0:3]
        now_dir = response[3:6]

while True:
    get_movement(client_sock)
    get_lrf_data(client_sock)

client_sock.close()