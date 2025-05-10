import socket
import pickle
import time

def send_object_to_server(obj, host='localhost', port=9400):
    # 序列化对象
    data = pickle.dumps(obj)
    
    # 创建socket连接
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        print(f"Connected to server {host}:{port}")
        
        # 发送数据
        s.sendall(data)
        print("Object sent to server")
        
        # 关闭写入端，让服务器知道数据已发送完
        s.shutdown(socket.SHUT_WR)
        
        # 接收结果
        result_data = b''
        while True:
            chunk = s.recv(4096)
            if not chunk:  # 接收到空数据表示结束
                break
            result_data += chunk
            print(f"Received chunk of size: {len(chunk)}")
        
        # 反序列化结果
        try:
            result = pickle.loads(result_data)
            if isinstance(result, dict) and 'error' in result:
                raise Exception(f"Server error: {result['error']}")
            return result
        except Exception as e:
            raise Exception(f"Error receiving result: {e}")

if __name__ == "__main__":
    your_object = {'name': 'ctx'}
    while True:
        try:
            result = send_object_to_server(your_object)
            print("Received result:", result)
        except Exception as e:
            print("Error:", e)
        time.sleep(3)