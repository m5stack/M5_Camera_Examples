import socket
import struct
import time

def run_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server_socket.bind(('0.0.0.0', 63333))  # 监听所有接口的63333端口
    server_socket.listen(1)
    
    print("等待ESP32连接...")
    
    while True:
        try:
            client_socket, addr = server_socket.accept()
            print(f"ESP32已连接: {addr}")
            
            image_count = 0
            
            while True:
                # 读取协议头 (7字节)
                header = b''
                while len(header) < 7:
                    chunk = client_socket.recv(7 - len(header))
                    if not chunk:
                        raise Exception("连接断开")
                    header += chunk
                
                # 解析协议头
                if header[0:3] == b'JPG':
                    data_len = struct.unpack('<I', header[3:7])[0]
                    print(f"接收到图像数据头，图像大小: {data_len} 字节")
                    
                    # 接收图像数据
                    image_data = b''
                    remaining = data_len
                    while remaining > 0:
                        chunk = client_socket.recv(min(4096, remaining))
                        if not chunk:
                            raise Exception("连接断开")
                        image_data += chunk
                        remaining -= len(chunk)
                    
                    # 保存图像
                    image_count += 1
                    filename = f"image_{image_count:04d}.jpg"
                    with open(filename, 'wb') as f:
                        f.write(image_data)
                    print(f"图像已保存为: {filename}")
                    
                    # 发送响应（可选）
                    response = f"Received image {image_count}".encode()
                    client_socket.send(response)
                else:
                    print("无效的协议头")
                    
        except Exception as e:
            print(f"错误: {e}")
            if 'client_socket' in locals():
                client_socket.close()
            continue
            
    server_socket.close()

if __name__ == "__main__":
    run_server()