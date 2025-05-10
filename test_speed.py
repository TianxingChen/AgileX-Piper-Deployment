import paramiko
import os
import time

def transfer_file_and_measure_speed(server_ip, username, private_key_path, port, local_file_path, remote_file_path):
    """
    通过 SSH 传输文件并测量传输速度
    :param server_ip: 远程服务器的IP地址
    :param username: 远程服务器的用户名
    :param private_key_path: 本地私钥文件路径
    :param port: 远程服务器的SSH端口
    :param local_file_path: 本地文件路径
    :param remote_file_path: 远程文件路径
    :return: 传输速度（MB/s）
    """
    # 创建 SSH 客户端
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        # 使用私钥连接到远程服务器
        ssh.connect(hostname=server_ip, username=username, key_filename=private_key_path, port=port)

        # 创建 SFTP 客户端
        sftp = ssh.open_sftp()

        # 获取文件大小
        file_size = os.path.getsize(local_file_path)
        print(f"文件大小: {file_size / (1024 * 1024):.2f} MB")

        # 开始传输文件
        start_time = time.time()
        sftp.put(local_file_path, remote_file_path)
        end_time = time.time()

        # 计算传输速度
        transfer_time = end_time - start_time
        transfer_speed = file_size / (transfer_time * 1024 * 1024)  # MB/s
        print(f"传输时间: {transfer_time:.2f} 秒")
        print(f"传输速度: {transfer_speed:.2f} MB/s")

        return transfer_speed

    except Exception as e:
        print(f"传输文件失败: {e}")
        return None

    finally:
        sftp.close()
        ssh.close()


if __name__ == "__main__":
    remote_server_ip = "8.130.84.63"  # 替换为你的远程服务器IP地址
    username = "root"  # 替换为你的远程服务器用户名
    private_key_path = os.path.expanduser("~/.ssh/id_rsa")  # 替换为你的私钥文件路径
    port = 904  # 替换为你的远程服务器SSH端口
    local_file_path = "./smallfile"  # 替换为本地大文件路径
    remote_file_path = "/mnt/workspace/chentianxing/smallfile"  # 替换为远程服务器上的目标路径

    # 传输文件并测量速度
    transfer_speed = transfer_file_and_measure_speed(remote_server_ip, username, private_key_path, port, local_file_path, remote_file_path)
    if transfer_speed is not None:
        print(f"传输速度: {transfer_speed:.2f} MB/s")