import socket
import struct
from math import *
from PIL import Image
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

while True:
    #clientInput= input(' please input > ')
    #if clientInput == "exit":
    #    break
    clientInput = '\n'
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

    img = plotPoint2Image( icp.ICPMatching( calc_local_coord(response)) , img)
    img.show()
    #print (response)

client_sock.close()