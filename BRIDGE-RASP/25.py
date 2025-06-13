import cv2
import numpy
import socket
import struct
import paho.mqtt.client as mqtt
from TfLunaI2C import TfLunaI2C
import serial
HOST = '192.168.65.91'  # Địa chỉ IP của laptop
PORT = 9999         # Cổng kết nối

# Tạo một socket TCP
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))  # Kết nối tới server
print('Now starting to send frames and distance data...')

# Khởi tạo UART với baudrate 9600
uart = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)  # Đổi "/dev/ttyAMA0" thành cổng serial của bạn
res = []
# MQTT Configuration
broker_address = "192.168.65.63"
topic = "robot/control"
def on_message(client, userdata, message):
    try:
        # Parse the received message (format: "w1,w2,w3,w4")
        data = message.payload.decode("utf-8")
        print(f"Received data via MQTT: {data[1]}")  # In dữ liệu ra terminal
        print(f"Received data via MQTT: {data[2]}")
        print(f"Received data via MQTT: {data[3]}")
        print(f"Received data via MQTT: {data[4]}") 
        # Split the received data into individual parameters
        clean_data = data.strip("[]").replace(" ", "")
        parameters = data.split(',')
        if len(parameters) == 4:
            # Prepare the UART message
            uart_message = f"{parameters[0]},{parameters[1]},{parameters[2]},{parameters[3]}\n"

            # Send the message via UART
            uart.write(uart_message.encode())
            print(f"Sent to UART: {uart_message}")  # In dữ liệu gửi qua UART
             # Read the response from UART
            uart_response = uart.readline().decode('utf-8').strip()
            res =  uart_response
            print(f"Received response from UART: {uart_response}")
        else:
            print("Invalid data format. Expected 4 parameters.")
        # return uart_response
    except Exception as e:
        print(f"Error handling MQTT message: {e}")


    

# Hàm đo khoảng cách
def measure_distance():
    tfluna = TfLunaI2C()
    tfluna.read_data()
    if tfluna.dist >= 0:  # Biên độ đáng tin cậy
        return tfluna.dist
    return -1  # Trả về -1 nếu biên độ không đáng tin cậy

# Lấy thiết bị camera
capture = cv2.VideoCapture(0)

mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect(broker_address)
mqtt_client.subscribe(topic)
mqtt_client.loop_start()

try:
    while True:
        # 1. Gửi dữ liệu khoảng cách
        distance = measure_distance()
        header = b'D'  # Header dữ liệu khoảng cách
        client.sendall(header + struct.pack('f', distance))  # Gửi khoảng cách dạng float
        print(f"Khoảng cách đã gửi: {distance} cm")
        # 3. Gửi dữ liệu khoảng cách
        # response = on_message()
        header = b'O'  # Header dữ liệu khoảng cách
        packed_data = struct.pack(f'{len(res)}f', *res)
        client.sendall(header + packed_data)
        # client.sendall(header + struct.pack('f', res))  # Gửi khoảng cách dạng float
        print(f" đã gửi: {res}")
        
        # 2. Gửi dữ liệu khung hình video
        success, frame = capture.read()
        while not success and frame is None:
            success, frame = capture.read()  # Lấy khung hình video

        # Mã hóa hình ảnh
        _, imgencode = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        img_data = imgencode.tobytes()

        # Gửi header và dữ liệu hình ảnh
        header = b'I'  # Header dữ liệu hình ảnh
        client.sendall(header + struct.pack('i', len(img_data)) + img_data)
        print("Khung hình đã gửi.")

        

except Exception as e:
    print(f"Lỗi: {e}")
finally:
    # Gửi chuỗi +0.00,+0.00,+0.00,+0.00 trước khi thoát
    reset_message = "+0.00,+0.00,+0.00,+0.00\n"
    try:
        uart.write(reset_message.encode())
        print(f"Sent reset message to UART: {reset_message}")
    except Exception as e:
        print(f"Error sending reset message: {e}")
    # Ngừng các kết nối và tài nguyên
    mqtt_client.loop_stop()  # Ngừng luồng MQTT
    mqtt_client.disconnect()  # Ngắt kết nối MQTT
    uart.close()
    capture.release()
    client.close()
    cv2.destroyAllWindows()
