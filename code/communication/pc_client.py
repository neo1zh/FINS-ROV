import socket
import keyboard

# 设置服务器的主机和端口
#server_host = '192.168.31.237'  # 监听所有网络接口
server_host = '192.168.137.25'
server_port = 65432

# 创建一个socket连接
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_host, server_port))
keys = ['w', 's', 'a', 'd', 'q', 'e', 'v']
try:
    while True:
        # 捕获键盘输入
        key_event = keyboard.read_event()
        
        # 检查是否按下了W、A、S、D键，并发送到服务器
        if key_event.event_type == keyboard.KEY_DOWN:
            if key_event.name in keys:
                # 发送键盘按键到服务器
                client_socket.send(key_event.name.encode())
except KeyboardInterrupt:
    print("客户端程序已停止。")
finally:
    client_socket.close()
