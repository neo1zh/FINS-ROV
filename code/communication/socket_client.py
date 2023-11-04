import socket
import cv2
import numpy as np
import time

class ReceiveImg(object):
    def __init__(self, host, port):
        # create socket and bind host
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)					# 设置创建socket服务的Client客服务的参数
        self.client_socket.connect((host, port))												# 连接的主机IP地址和端口
        self.connection = self.client_socket.makefile('rb')										# 创建一个makefile传输文件，读功能，读数据是b''二进制类型
        print(" ")
        print("已连接到服务端：")
        print("Host : ", host)
        print("请按‘q’退出图像传输!")
        # print("（提示：需要退出时，先按‘Ctrl’键再按‘q’键！）")
        print(" ")
        self.start()

    def start(self):
        duration = 0.01
        try:
            # need bytes here
            stream_bytes = b' '											# 创建一个变量，存放的数据类型是b''二进制类型数据
            while True:
                time_past = time.time()
                msg = self.connection.read(1024)						# 读makefile传输文件，一次读1024个字节
                stream_bytes += msg
                first = stream_bytes.find(b'\xff\xd8')					# 检测帧头位置
                last = stream_bytes.find(b'\xff\xd9')					# 检测帧尾位置

                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]					# 帧头和帧尾中间的数据就是二进制图片数据（编码后的二进制图片数据，需要解码后使用）
                    stream_bytes = stream_bytes[last + 2:]				# 更新stream_bytes数据
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)			# 将二进制图片数据转换成numpy.uint8格式（也就是图片格式）数据，然后解码获得图片数据

                    time_now = time.time()
                    
                    duration = 0.5*duration+0.5*(time_now-time_past)
                    
                    text = "fps: "+str(1/duration)[:4]+"  Shape: "+str(image.shape[0])+","+str(image.shape[1])  ##编辑文本
 
                    fontScale = 1  # 字体缩放比例
 
                    color = (0, 255, 0)  # 字体颜色
 
                    pos = (10, 50)  #位置
 
                    image = cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, fontScale, color)


                    cv2.imshow('image', image)							# 打开一个窗口显示图片 
                    if cv2.waitKey(1) & 0xFF == ord('q'):				# 等待1ms后显示下一帧图片
                        break
        except Exception as e:
            print(e)
            print("Error：连接出错！")

        finally:
            print("已退出图像传输！")
            cv2.destroyAllWindows()
            print("已关闭窗口！")
            self.connection.close()
            print("与断开服务端连接！")


if __name__ == '__main__':
    ReceiveImg('192.168.31.37', 8000)
