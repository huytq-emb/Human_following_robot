import paho.mqtt.client as mqtt
import time
import json  # Dùng để chuyển đổi mảng thành chuỗi JSON
# Cấu hình MQTT
# broker_address = "192.168.65.63"  # Địa chỉ IP của broker MQTT (thường là IP của Raspberry Pi)
broker_address = "192.168.144.90"  # Địa chỉ IP của broker MQTT (thường là IP của Raspberry Pi)
topic = "robot/control"          # Chủ đề MQTT để gửi tín hiệu điều khiển
topic1 = "servo/control"          
# topic1 = "omega/control"
# Kết nối tới MQTT broker
client = mqtt.Client()
client.connect(broker_address)

# Hàm gửi tín hiệu điều khiển tới robot



def send_control_signal(direction):
    """
    Gửi tín hiệu điều khiển tới robot thông qua MQTT
    direction: lệnh điều khiển (ví dụ: 'forward', 'backward', 'left', 'right', 'stop')
    """
    client.publish(topic, direction)
    print(f"Sent control signal: {direction}")

def send_camera(frame):
    client.publish(topic, frame)
    time.sleep(0.1)  # Gửi frame mỗi 100ms
    print("Frame đã được gửi.")

def send_servo(ser):
    """
    Gửi tín hiệu điều khiển tới robot thông qua MQTT
    direction: lệnh điều khiển (ví dụ: 'forward', 'backward', 'left', 'right', 'stop')
    """
    # omega_json = json.dumps(omega_array)
    client.publish(topic1, ser)
    print(f"Sent control signal: {ser}")
