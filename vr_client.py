import socket

# กำหนด IP และพอร์ตของ Server ที่ Unity เปิดไว้
SERVER_IP = "10.61.2.1"
SERVER_PORT = 5555
BUFFER_SIZE = 1024  # ขนาด buffer สำหรับรับข้อมูล

def main():
    # สร้าง TCP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # เชื่อมต่อกับ Server
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to server at {SERVER_IP}:{SERVER_PORT}")

        while True:
            # รอรับข้อมูล
            data = client_socket.recv(BUFFER_SIZE)

            if not data:
                print("Disconnected from server.")
                break

            # แปลงข้อมูลจาก byte เป็น string
            message = data.decode('utf-8')
            print("Received:", message)

    except ConnectionRefusedError:
        print("Connection refused. Is the server running?")
    except Exception as e:
        print("Error:", e)
    finally:
        client_socket.close()

if __name__ == "__main__":
    main()
