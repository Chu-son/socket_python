# -*- coding:utf-8 -*-
import socket

host = "localhost" #���g���̃T�[�o�[�̃z�X�g�������܂�
port = 8000 #�N���C�A���g�Ɠ���PORT�����Ă����܂�

serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
serversock.bind((host,port)) #IP��PORT���w�肵�ăo�C���h���܂�
serversock.listen(10) #�ڑ��̑҂��󂯂����܂��i�L���[�̍ő吔���w��j

print( 'Waiting for connections...')
clientsock, client_address = serversock.accept() #�ڑ������΃f�[�^���i�[

while True:
    rcvmsg = clientsock.recv(1024)
    print( 'Received -> %s' % (rcvmsg))
    if rcvmsg == '':
      break
    print( 'Type message...')
    s_msg = raw_input()
    if s_msg == '':
      break
    print( 'Wait...')

    clientsock.sendall(s_msg) #���b�Z�[�W��Ԃ��܂�
clientsock.close()