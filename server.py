import socket
import pickle

def start_server(host='0.0.0.0', port=9400):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 防止地址占用
        s.bind((host, port))
        s.listen()
        print(f"Server listening on {host}:{port}")
        
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                
                # 接收数据
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:  # 连接关闭
                        break
                    data += chunk
                    print(f"Received chunk of size: {len(chunk)}")
                
                if not data:
                    print("Received empty data")
                    continue
                
                # 反序列化对象
                try:
                    received_obj = pickle.loads(data)
                    print("Object received successfully:", received_obj)
                    
                    # 处理对象（模拟处理）
                    result = ['ok']
                    
                    # 发送结果
                    result_data = pickle.dumps(result)
                    conn.sendall(result_data)
                    print("Result sent back to client")
                    
                except Exception as e:
                    print(f"Error processing object: {e}")
                    error_data = pickle.dumps({"error": str(e)})
                    conn.sendall(error_data)

if __name__ == "__main__":
    start_server()