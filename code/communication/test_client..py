import socket
import cv2
import numpy as np
import time
import threading
import keyboard

# 设置服务器的主机和端口
server_host = '192.168.137.25'
keyboard_port = 65432
video_host = '192.168.137.25'
video_port = 8000

keys = ['w', 's', 'a', 'd', 'q', 'e', 'v']

class KeyboardControlClient:
    def __init__(self, host, port):
        self.server_host = host
        self.server_port = port

        # 创建一个socket连接
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_host, self.server_port))

    def start(self):
        try:
            while True:
                # 捕获键盘输入
                key_event = keyboard.read_event()
                
                # 检查是否按下了W、A、S、D键，并发送到服务器
                if key_event.event_type == keyboard.KEY_DOWN:
                    if key_event.name in keys:
                        # 发送键盘按键到服务器
                        self.client_socket.send(key_event.name.encode())
        except KeyboardInterrupt:
            print("客户端程序已停止。")
        finally:
            self.client_socket.close()

class ReceiveImg:
    def __init__(self, host, port):
        # 创建socket和绑定主机
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host, port))
        self.connection = self.client_socket.makefile('rb')

    def start(self):
        duration = 0.01
        try:
            stream_bytes = b' '
            while True:
                time_past = time.time()
                msg = self.connection.read(1024)
                stream_bytes += msg
                first = stream_bytes.find(b'\xff\xd8')
                last = stream_bytes.find(b'\xff\xd9')

                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]
                    stream_bytes = stream_bytes[last + 2:]
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                    time_now = time.time()
                    
                    duration = 0.5 * duration + 0.5 * (time_now - time_past)
                    
                    text = "fps: " + str(1 / duration)[:4] + "  Shape: " + str(image.shape[0]) + "," + str(image.shape[1])
                    fontScale = 1
                    color = (0, 255, 0)
                    pos = (10, 50)
                    image = cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, fontScale, color)

                    cv2.imshow('image', image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except Exception as e:
            print(e)
            print("Error：连接出错！")
        finally:
            print("已退出图像传输！")
            cv2.destroyAllWindows()
            print("已关闭窗口！")
            self.connection.close()

if __name__ == '__main__':
    keyboard_control = KeyboardControlClient(server_host, keyboard_port)
    video_receiver = ReceiveImg(video_host, video_port)

    keyboard_thread = threading.Thread(target=keyboard_control.start)
    video_thread = threading.Thread(target=video_receiver.start)

    keyboard_thread.start()
    video_thread.start()

    keyboard_thread.join()
    video_thread.join()
