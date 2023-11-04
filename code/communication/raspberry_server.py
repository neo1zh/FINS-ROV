import socket
import serial
import utils

# 串口配置参数
ser = serial.Serial(
    port="/dev/ttyAMA1",  # 根据实际串口连接选择
    baudrate=115200,       # 波特率，根据需要设置
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1  # 读取超时时间，根据需要设置
)

keys = ['w', 's', 'a', 'd', 'q', 'e', 'v']

# 其他服务器配置
server_host = '192.168.137.25'
server_port = 65432

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_host, server_port))
server_socket.listen(1)
print(f"等待客户端连接在 {server_host}:{server_port}...")

client_socket, client_address = server_socket.accept()
print(f"连接来自 {client_address[0]}:{client_address[1]}")

try:
    while True:
        key = client_socket.recv(1).decode()
        if key in keys:
            print(f"收到键盘输入: {key}")
            
            # 使用key2cmd函数将键盘输入转换为PWM值的字符串
            pwm_cmd = utils.key2cmd(key)

            # 将接收到的键盘输入发送到串口
            ser.write(pwm_cmd.encode())
        else:
            print(f"无效的键盘输入: {key}")
except KeyboardInterrupt:
    print("服务器程序已停止。")
finally:
    client_socket.close()
    server_socket.close()
    ser.close()

