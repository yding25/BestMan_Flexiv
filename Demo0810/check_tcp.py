"""
这个代码的功能是创建一个简单的网络通信系统,包括数据的发送和接收功能。
接收的数据端口是9088,如果收到指令"arm.make_coffee()",会运行一个名为make_coffee.py的脚本,并发送相关通知。
注意, 双方电脑必须在同一个wifi里。
对方电脑: 192.168.31.8
本地电脑: 192.168.31.47

Run this script using:
python main.py
"""

import socket  # 导入socket模块用于网络通信
import time  # 导入time模块用于添加延迟
import subprocess  # 导入subprocess模块用于运行外部脚本
import threading  # 导入threading模块用于处理多线程

def send_data(ip_address, send_port, message):
    """
    函数:向指定的IP地址和端口发送数据
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建一个TCP/IP套接字
    if sock.fileno() < 0:
        print("创建套接字失败。")
        return

    # 设置服务器地址
    server_address = (ip_address, send_port)

    # 连接到服务器
    while True:
        try:
            sock.connect(server_address)  # 尝试连接到服务器
            break
        except socket.error as e:
            print("连接服务器失败。重试中...")
            time.sleep(0.1)  # 等待后重试
            continue

    # 发送数据
    try:
        sock.sendall(message.encode())  # 发送消息
    except socket.error as e:
        print("发送数据失败。")
    
    sock.close()  # 关闭套接字

def receive_data():
    """
    函数:接收客户端数据
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建一个TCP/IP套接字
    if server_socket.fileno() < 0:
        print("创建套接字失败。")
        return ""

    # 设置服务器地址
    server_address = ('', 9088)  # 监听所有可用接口上的9088端口

    # 绑定服务器地址和端口
    try:
        server_socket.bind(server_address)  # 将套接字绑定到地址
    except socket.error as e:
        print("绑定套接字失败。")
        server_socket.close()  # 绑定失败时关闭套接字
        return ""

    # 监听连接请求
    server_socket.listen(1)  # 监听传入连接,最大等待队列为1
    print("服务器正在监听连接...")

    while True:
        # 接受客户端连接
        client_socket, client_address = server_socket.accept()  # 等待连接
        if client_socket.fileno() < 0:
            print("接受连接失败。")
            continue

        # 接收数据
        buffer_size = 1024
        buffer = client_socket.recv(buffer_size).decode()  # 接收客户端数据
        
        if not buffer:
            print("接收数据失败。")
        else:
            print("接收到的数据:", buffer)
            # if buffer.strip() == "arm.make_coffee()":  # 检查接收到的命令是否是制作咖啡
            #     print("正在执行make_coffee.py脚本...")
            #     send_data("192.168.31.8", 8028, "start")  # 通知'对方电脑'过程开始
                
            #     # 运行make_coffee.py的脚本
            #     param1 = "192.168.2.100" # robot ip
            #     param2 = "192.168.2.108" # local ip
            #     param3 = "20" # 指令发送频率
            #     subprocess.run(["python", "make_coffee.py", param1, param2, param3])  # 运行make_coffee.py脚本并传递参数
                
            #     # 使用send_data发送确认信息回客户端
            #     send_data("192.168.31.8", 8028, "true")  # 通知'对方电脑'过程成功完成

        client_socket.close()  # 可选地关闭客户端套接字

def start_receiving():
    """
    函数:启动一个新线程接收数据
    """
    receive_thread = threading.Thread(target=receive_data)  # 创建一个新的线程用于receive_data
    receive_thread.daemon = True  # 将线程设置为守护线程
    receive_thread.start()  # 启动线程

if __name__ == "__main__":
    start_receiving()  # 启动接收数据的线程
    # 主线程可以继续运行其他代码
    while True:
        time.sleep(1)  # 保持主线程运行