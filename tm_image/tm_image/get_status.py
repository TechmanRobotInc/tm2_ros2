import socket

from threading import Thread

import time

from rclpy.clock import Clock
from rclpy.node import Node

from tm_image import translate_json_to_list


class Talker(Node):
    def __init__(self, ip):
        super().__init__('talker')
        self.ip = ip
        self.prot = 5891
        self.socketConnect = None
        self.isConnect = False

    def socket_connect(self):
        self.socketConnect = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socketConnect.connect((self.ip, self.prot))
        self.isConnect = True
        self.backgroundListen = Thread(target=self.listener_callback)
        self.backgroundListen.start()

    def listener_callback(self):
        remainString = ''
        while self.isConnect:
            dataByte = self.socketConnect.recv(1024)
            print('get data from server which is')
            data = str(dataByte, encoding='utf-8')
            print(data)
            data = data+remainString
            print('new data is')
            remainString, newString = translate_json_to_list.TmJsonToDiction.split_package(data)
            if(newString is not None):
                print(newString[0])
                jsonString = translate_json_to_list.TmJsonToDiction.tm_string_to_json(newString[0])
                print(jsonString)
                dictionary = translate_json_to_list.TmJsonToDiction.json_to_dict(jsonString)
                print(dictionary)


def main(args=None):
    time_stamp = Clock().now()
    print(time)
    msg = time_stamp.to_msg()
    print('Seconds since epoch =', msg)


if __name__ == '__main__':
    main()
