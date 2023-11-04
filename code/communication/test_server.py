import numpy as np
import cv2
import socket
import io
import struct
import time
import threading
import serial
import keyboard
import utils

class VideoStreamingServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.server_socket = None
        self.connection = None
        self.connect = None

    def initialize(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        print(f"等待客户端连接在 {self.host}:{self.port}...")
        self.connection, self.client_address = self.server_socket.accept()
        self.connect = self.connection.makefile('wb')

    def start(self):
        try:
            self.initialize()
            print(f"连接来自 {self.client_address[0]}:{self.client_address[1]}")
            print("Streaming...")
            camera = cv2.VideoCapture(0)
            stream = io.BytesIO()

            while True:
                ret, frame = camera.read()
                if cv2.waitKey(1) == 27:
                    break
                
                img_encode = cv2.imencode('.jpg', frame)[1]
                data_encode = np.array(img_encode)
                stream.write(data_encode)
                
                self.connect.write(struct.pack('<L', stream.tell()))
                self.connect.flush()
                
                stream.seek(0)
                self.connect.write(stream.read())
                stream.seek(0)
                stream.truncate()
                
            self.connect.write(struct.pack('<L', 0))
        except Exception as e:
            print(e)
            print(" ")
            print("The customer service is disconnected!")

        finally:
            cv2.destroyAllWindows()
            self.connection.close()
            print("与客户端断开连接!")
            print(" ")

class KeyboardControlServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.ser = serial.Serial(
            port="/dev/ttyAMA1",  # 根据实际串口连接选择
            baudrate=115200,       # 波特率，根据需要设置
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1  # 读取超时时间，根据需要设置
        )
        self.keys = ['w', 's', 'a', 'd', 'q', 'e', 'v']

    def start(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        print(f"等待客户端连接在 {self.host}:{self.port}...")

        client_socket, client_address = server_socket.accept()
        print(f"连接来自 {client_address[0]}:{client_address[1]}")

        try:
            while True:
                key = client_socket.recv(1).decode()
                if key in self.keys:
                    print(f"收到键盘输入: {key}")

                    pwm_cmd = utils.key2cmd(key)
                    self.ser.write(pwm_cmd.encode())
                else:
                    print(f"无效的键盘输入: {key}")
        except KeyboardInterrupt:
            print("服务器程序已停止。")
        finally:
            client_socket.close()
            server_socket.close()
            self.ser.close()

if __name__ == '__main__':
    video_server = VideoStreamingServer('192.168.137.25', 8000)
    keyboard_server = KeyboardControlServer('192.168.137.25', 65432)

    video_thread = threading.Thread(target=video_server.start)
    keyboard_thread = threading.Thread(target=keyboard_server.start)

    video_thread.start()
    keyboard_thread.start()

    video_thread.join()
    keyboard_thread.join()
