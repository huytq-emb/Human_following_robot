import cv2
import numpy as np
import socket
import struct
import threading
import time
import csv
import mediapipe as mp
from collections import deque
from ultralytics import YOLO
from sort import Sort 

from send import send_control_signal  # Import hàm MQTT từ control_robot.py
data_to_write = deque(maxlen=10000)
file_name = 'lap9.csv'
############################################
# 1) CÁC BIẾN & THAM SỐ TOÀN CỤC
############################################

# Socket server
HOST = '0.0.0.0'
PORT = 9999

# Khởi tạo server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
print("[INFO] Waiting for connection...")
conn, addr = server.accept()
print(f"[INFO] Connected to {addr}")

# Hàng đợi chứa frame
frame_queue = deque(maxlen=2)  # Lưu tối đa 2 frame mới nhất
distance = 9999.0
distance_lock = threading.Lock()
distance_buffer = []

# Mediapipe Hands
mp_hands = mp.solutions.hands

# YOLO
model_yolo = YOLO('yolov8n.pt', task='detect').to('cuda')
model_yolo.conf = 0.5
model_yolo.iou = 0.45
model_yolo.classes = [0]  # chỉ detect 'person'

# SORT multi-object tracker
sort_tracker = Sort()

# CSRT single-object tracker
csrt_tracker = None

# Trạng thái Admin
admin_confirmed = False
target_id = None  # ID của Admin bên SORT
last_admin_center = None  # Lưu vị trí trung tâm cơ thể cuối của Admin

# Các thông số cho skip frame YOLO
frame_count = 0
SKIP_FRAME_COUNT = 3

# Thông số tracking / điều khiển robot
DEAD_ZONE_X = 20
DEAD_ZONE_D = 30
VX_MAX = 0.44
OMEGA_MAX = 1.0
desired_distance = 150.0  # cm
KP_dist = 1
KP_omega = 1.5
# KP_omega = 2.0

# Mô hình Mecanum (nếu bạn cần tính wheel velocity)
R = 0.096
L1 = 0.16
L2 = 0.13

# Bộ đệm để lọc trung bình trượt
centerX_buffer = []
centerY_buffer = []

# Trạng thái hệ thống: "đang Tracking" hay "đang Search"
STATE_TRACKING = 0
STATE_SEARCHING = 1
current_state = STATE_TRACKING

# Thông số "Auto-search"
search_direction = None  # 'left' hoặc 'right'
search_start_time = None
SEARCH_TIMEOUT = 10.0    # Thời gian tối đa robot xoay để tìm
OMEGA_SEARCH = 0.5       # Tốc độ quay khi search (rad/s)

############################################
# 2) LỚP BodyCenterDetector
############################################

class BodyCenterDetector:
    def __init__(self, min_detection_confidence=0.5, min_tracking_confidence=0.5):
        # Khởi tạo Mediapipe Pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
    
    def calculate_center(self, landmarks, frame_width, frame_height):
        # Chọn các điểm mấu quan trọng để tính trung tâm, ví dụ: vai trái, vai phải, hông trái, hông phải
        key_points = [
            landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value],
            landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value],
            landmarks[self.mp_pose.PoseLandmark.LEFT_HIP.value],
            landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP.value]
        ]
        
        x_coords = [int(point.x * frame_width) for point in key_points]
        y_coords = [int(point.y * frame_height) for point in key_points]
        
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
        
        return center_x, center_y
    
    def process_frame(self, frame):
        """
        Xử lý một khung hình để tìm trung tâm cơ thể.
        
        Args:
            frame (numpy.ndarray): Khung hình đầu vào (BGR).
        
        Returns:
            processed_frame (numpy.ndarray): Khung hình đã được xử lý với trung tâm được đánh dấu.
            center_x (int or None): Tọa độ x của trung tâm nếu phát hiện, ngược lại là None.
            center_y (int or None): Tọa độ y của trung tâm nếu phát hiện, ngược lại là None.
        """
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        # Phát hiện các điểm mấu cơ thể
        results = self.pose.process(image)
        
        # Chuyển đổi lại sang BGR để hiển thị
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        frame_height, frame_width = image.shape[:2]
        center_x, center_y = None, None
        
        if results.pose_landmarks: 
            # Tính toán trung tâm cơ thể
            center_x, center_y = self.calculate_center(results.pose_landmarks.landmark, frame_width, frame_height)
        return image, center_x, center_y
    
    def close(self):
        self.pose.close()

############################################
# 3) HÀM HỖ TRỢ
############################################

def moving_average_filter(new_val, buffer, size=5):
    buffer.append(new_val)
    if len(buffer) > size:
        buffer.pop(0)
    return sum(buffer) / len(buffer)

def format_data(numbers):
    """
    Ví dụ: [0.3, -0.22, 0.1, -0.45] => '+0.30,-0.22,+0.10,-0.45'
    """
    formatted_numbers = []
    for num in numbers:
        sign = '+' if num >= 0 else '-'
        formatted_numbers.append(f"{sign}{abs(num):.2f}")
    return ",".join(formatted_numbers)

def run_yolo_on_frame(frame):
    results = model_yolo.predict(frame, verbose=False)
    boxes = results[0].boxes
    detections_for_sort = []
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        conf = box.conf[0]
        if conf < 0.5:
            continue
        detections_for_sort.append([float(x1), float(y1),
                                    float(x2), float(y2),
                                    float(conf)])
    return detections_for_sort


class VSignDetector:
    """
    Lớp để nhận diện cử chỉ V sign sử dụng Mediapipe Hands.
    """
    def __init__(self, 
                 max_num_hands=1, 
                 detection_confidence=0.5, 
                 tracking_confidence=0.5):
        """
        Khởi tạo đối tượng VSignDetector.
        
        Args:
            max_num_hands (int): Số lượng tay tối đa cần phát hiện.
            detection_confidence (float): Ngưỡng độ tin cậy phát hiện.
            tracking_confidence (float): Ngưỡng độ tin cậy theo dõi.
        """
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=detection_confidence,
            min_tracking_confidence=tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils

    def is_v_sign(self, img_bgr):
        """
        Kiểm tra trong ảnh img_bgr có cử chỉ V Sign hay không.
        
        Args:
            img_bgr (numpy.ndarray): Ảnh đầu vào ở định dạng BGR.
        
        Returns:
            bool: True nếu phát hiện V Sign, ngược lại False.
        """
        if img_bgr is None or img_bgr.size == 0:
            return False

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)

        if not results.multi_hand_landmarks:
            return False

        for hand_landmarks in results.multi_hand_landmarks:
            # Lấy tọa độ các điểm mấu của ngón tay
            fingers = self._get_finger_status(hand_landmarks)
            # Kiểm tra điều kiện để xác định cử chỉ V Sign
            if fingers['index'] and fingers['middle'] and not fingers['ring'] and not fingers['pinky']:
                return True

        return False

    def _get_finger_status(self, hand_landmarks):
        """
        Xác định trạng thái (duỗi hay gập) của các ngón tay.
        
        Args:
            hand_landmarks (mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList): Dữ liệu điểm mấu tay.
        
        Returns:
            dict: Trạng thái của từng ngón tay.
        """
        # Chỉ số các điểm mấu cho các ngón tay
        finger_tips_ids = {
            'thumb': 4,
            'index': 8,
            'middle': 12,
            'ring': 16,
            'pinky': 20
        }

        finger_pip_ids = {
            'thumb': 2,  
            'index': 6,
            'middle': 10,
            'ring': 14,
            'pinky': 18
        }

        fingers = {}

        for finger in ['index', 'middle', 'ring', 'pinky']:
            tip_y = hand_landmarks.landmark[finger_tips_ids[finger]].y
            pip_y = hand_landmarks.landmark[finger_pip_ids[finger]].y
            fingers[finger] = tip_y < pip_y
        fingers['thumb'] = False

        return fingers

    def draw_hand_landmarks(self, img_bgr, hand_landmarks):
        """
        Vẽ các điểm mấu và kết nối của tay lên ảnh.
        
        Args:
            img_bgr (numpy.ndarray): Ảnh đầu vào ở định dạng BGR.
            hand_landmarks (mediapipe.framework.formats.landmark_pb2.NormalizedLandmarkList): Dữ liệu điểm mấu tay.
        """
        self.mp_draw.draw_landmarks(img_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

    def close(self):
        """
        Đóng tài nguyên của Mediapipe Hands.
        """
        self.hands.close()

def send_drive_command(vx, vy, omega):
    """
    Hàm điều khiển robot (thay thế cho send_control_signal wheel velocities).
    Ví dụ: Mecanum => Tính wheel velocity. Ở đây ta chỉ in demo.
    """
    # Tính wheel velocity (mẫu Mecanum)
    J = np.array([
        [1, -1, -(L1+L2)],
        [1,  1,  (L1+L2)],
        [1,  1, -(L1+L2)],
        [1, -1,  (L1+L2)]
    ])
    state = np.array([vx, vy, omega])
    wheel_velocities = np.dot(J, state) / R
    wheel_velocities = np.clip(wheel_velocities, -3.5, 3.5)

    formatted_data = np.round(wheel_velocities, 1)
    data_to_write.append(formatted_data)

    # Ghi dữ liệu vào file
    with open(file_name, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in data_to_write:
            writer.writerow(row)
    # Format
    data_str = format_data(wheel_velocities)
    send_control_signal(data_str)

def detect_v_sign_and_init_csrt(frame, body_center_detector, vsign_detector):
    """
    Nếu chưa có admin, ta chạy YOLO+SORT => ai có V Sign => init CSRT => set admin.
    Trả về True nếu phát hiện ra Admin, False nếu không.
    """
    global admin_confirmed, target_id, csrt_tracker

    detections_for_sort = run_yolo_on_frame(frame)
    if detections_for_sort:
        tracked_objs = sort_tracker.update(np.array(detections_for_sort))
    else:
        tracked_objs = sort_tracker.update()

    H, W = frame.shape[:2]

    for det in tracked_objs:
        x1, y1, x2, y2, tid = det
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        x1 = max(0, min(x1, W))
        x2 = max(0, min(x2, W))
        y1 = max(0, min(y1, H))
        y2 = max(0, min(y2, H))

        person_crop = frame[y1:y2, x1:x2]
        if vsign_detector.is_v_sign(person_crop):
            admin_confirmed = True
            target_id = tid

            # Mở rộng bounding box thêm 20% để bao phủ toàn bộ cơ thể
            w = x2 - x1
            h = y2 - y1
            padding = 0.2
            x1_padded = max(0, int(x1 - w * padding))
            y1_padded = max(0, int(y1 - h * padding))
            x2_padded = min(W, int(x2 + w * padding))
            y2_padded = min(H, int(y2 + h * padding))

            csrt_tracker = cv2.TrackerCSRT_create()
            csrt_tracker.init(frame, (x1_padded, y1_padded, x2_padded - x1_padded, y2_padded - y1_padded))
            print(f"[INFO] Admin selected => ID={target_id}")
            return True

    return False

def re_detect_admin(frame, admin_id, body_center_detector):
    """
    Chạy YOLO+SORT để tìm bounding box track_id = admin_id.
    Nếu tìm thấy => khởi tạo CSRT => trả về True, else False.
    """
    global admin_confirmed, csrt_tracker

    detections_for_sort = run_yolo_on_frame(frame)
    if detections_for_sort:
        re_tracked = sort_tracker.update(np.array(detections_for_sort))
    else:
        re_tracked = sort_tracker.update()

    H, W = frame.shape[:2]
    found_admin = False
    for det in re_tracked:
        x1, y1, x2, y2, tid = det
        if tid == admin_id:
            # Vẫn trong khung => init lại CSRT
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            x1 = max(0, min(x1, W))
            x2 = max(0, min(x2, W))
            y1 = max(0, min(y1, H))
            y2 = max(0, min(y2, H))

            # Mở rộng bounding box thêm 20%
            w = x2 - x1
            h = y2 - y1
            padding = 0.2
            x1_padded = max(0, int(x1 - w * padding))
            y1_padded = max(0, int(y1 - h * padding))
            x2_padded = min(W, int(x2 + w * padding))
            y2_padded = min(H, int(y2 + h * padding))

            csrt_tracker = cv2.TrackerCSRT_create()
            csrt_tracker.init(frame, (x1_padded, y1_padded, x2_padded - x1_padded, y2_padded - y1_padded))
            admin_confirmed = True
            found_admin = True
            print(f"[INFO] Re-init CSRT for Admin ID={admin_id}")
            break
    return found_admin

############################################
# 4) THREAD A: NHẬN DỮ LIỆU TỪ SOCKET
############################################

def receiver_thread(conn):
    global distance
    try:
        while True:
            header = conn.recv(1)
            if not header:
                print("[INFO] Socket closed (header).")
                break

            if header == b'D':
                # Nhận distance
                data = conn.recv(4)
                if not data:
                    break
                dist_val = struct.unpack('f', data)[0]
                with distance_lock:
                    distance = moving_average_filter(dist_val, distance_buffer, size=5)

            elif header == b'I':
                # Nhận ảnh
                length_data = conn.recv(4)
                if len(length_data) != 4:
                    print("[WARN] length_data != 4 bytes.")
                    continue

                length = struct.unpack('i', length_data)[0]
                data = b""
                while len(data) < length:
                    packet = conn.recv(length - len(data))
                    if not packet:
                        print("[WARN] Incomplete frame data.")
                        break
                    data += packet

                np_data = np.frombuffer(data, dtype=np.uint8)
                frame = cv2.imdecode(np_data, 1)
                if frame is not None:
                    frame_queue.append(frame)

            else:
                # Header không mong đợi => bỏ qua
                pass

    except Exception as e:
        print(f"[ERROR] Receiver thread: {e}")
    finally:
        conn.close()
        print("[INFO] Receiver thread stopped.")

############################################
# 5) THREAD B: XỬ LÝ CHÍNH
############################################

def processing_thread(body_center_detector,vsign_detector):
    global frame_count, admin_confirmed, target_id, csrt_tracker
    global current_state, search_direction, search_start_time, last_admin_center

    while True:
        if len(frame_queue) == 0:
            time.sleep(0.01)
            continue
        frame = frame_queue.pop()

        with distance_lock:
            current_distance_local = distance

        H, W = frame.shape[:2]

        # -------------------------------------
        # PHÂN LUỒNG THEO state: TRACKING / SEARCHING
        # -------------------------------------
        if current_state == STATE_TRACKING:
            # A) TRẠNG THÁI TRACKING
            if admin_confirmed and csrt_tracker is not None:
                # Đã có Admin => CSRT update
                success, bbox = csrt_tracker.update(frame)
                if success:
                    x, y, w, h = [int(v) for v in bbox]
                    # Tính trung tâm cơ thể bằng BodyCenterDetector
                    processed_frame, center_x, center_y = body_center_detector.process_frame(frame)
                    if center_x is not None and center_y is not None:
                        body_center = (center_x, center_y)
                        last_admin_center = body_center
                        # Vẽ điểm trung tâm đã được thực hiện trong lớp
                        # Bạn có thể thêm các xử lý khác nếu cần
                    else:
                        body_center = None
                        print('Body Center not detected.')

                    # Vẽ bounding box
                    cv2.rectangle(processed_frame, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(processed_frame, f"ADMIN({target_id})", (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                    # Tính vận tốc dựa trên trung tâm cơ thể
                    if last_admin_center:
                        admin_cX, admin_cY = last_admin_center
                        error_x = (W / 2) - admin_cX
                        norm_error_x = error_x / (W / 2)
                        error_d = desired_distance - current_distance_local

                        if abs(error_d) < DEAD_ZONE_D:
                            error_d = 0
                        if abs(norm_error_x) < (DEAD_ZONE_X / (W / 2)):
                            norm_error_x = 0

                        vx = -KP_dist * error_d * 0.01
                        vx = np.clip(vx, -VX_MAX, VX_MAX)
                        vy = 0
                        omega = KP_omega * norm_error_x *1.25
                        omega = np.clip(omega, -OMEGA_MAX, OMEGA_MAX)

                        # Nếu quá gần => dừng
                        if current_distance_local < desired_distance + DEAD_ZONE_D:
                            vx, omega = 0, 0
                            print("[INFO] Too close => stop.")
                        send_drive_command(vx, vy, omega)

                    # Hiển thị frame đã xử lý
                    cv2.imshow("Frame", processed_frame)
                else:
                    # CSRT mất dấu => re-detect ID cũ
                    print("[WARN] CSRT lost => re-detect ID=", target_id)
                    success_re = re_detect_admin(frame, target_id, body_center_detector)
                    if not success_re:
                        # => Không thấy admin => chuyển sang SEARCH
                        current_state = STATE_SEARCHING
                        search_start_time = time.time()
                        # Xác định hướng xoay
                        if last_admin_center is not None:
                            admin_cX, _ = last_admin_center
                            if admin_cX < W / 2:
                                search_direction = 'left'
                            else:
                                search_direction = 'right'
                        else:
                            search_direction = 'right'
                        admin_confirmed = False
                        csrt_tracker = None
                        print("[INFO] Switch to SEARCHING state.")
                    # Hiển thị frame gốc nếu không thành công
                    cv2.imshow("Frame", frame)

            else:
                # Chưa có admin => skip frame YOLO
                if frame_count % SKIP_FRAME_COUNT == 0:
                    found = detect_v_sign_and_init_csrt(frame, body_center_detector,vsign_detector)
                    # found => nếu cử chỉ V => admin_confirmed = True, ...
                frame_count += 1

                # Robot ko có admin => tạm dừng
                send_drive_command(0, 0, 0)
                # Hiển thị frame đã xử lý
                cv2.imshow("Frame", frame)

        elif current_state == STATE_SEARCHING:
            # B) TRẠNG THÁI SEARCHING
            # Robot tự xoay sang trái/phải
            if search_direction == 'left':
                omega = +OMEGA_SEARCH
            else:
                omega = -OMEGA_SEARCH
            vx = 0
            vy = 0
            send_drive_command(vx, vy, omega)

            # Check timeout
            elapsed = time.time() - search_start_time
            if elapsed > SEARCH_TIMEOUT:
                print("[WARN] Search timeout => stop.")
                send_drive_command(0,0,0)
                current_state = STATE_TRACKING
                admin_confirmed = False
                target_id = None
                continue

            # Trong lúc xoay => YOLO + SORT => tìm người giữa khung => chờ V Sign
            detections_for_sort = run_yolo_on_frame(frame)
            if detections_for_sort:
                tracked_objs = sort_tracker.update(np.array(detections_for_sort))
            else:
                tracked_objs = sort_tracker.update()

            found_center_person = None
            for det in tracked_objs:
                x1, y1, x2, y2, tid = det
                cX = (x1 + x2) / 2
                if (cX >= W / 3) and (cX <= 2 * W / 3):
                    found_center_person = det
                    break

            if found_center_person is not None:
                # Dừng xoay
                send_drive_command(0,0,0)
                x1, y1, x2, y2, tid = found_center_person
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                x1 = max(0, min(x1, W))
                x2 = max(0, min(x2, W))
                y1 = max(0, min(y1, H))
                y2 = max(0, min(y2, H))

                person_crop = frame[y1:y2, x1:x2]
                if vsign_detector.is_v_sign(person_crop):
                    # => Admin mới
                    admin_confirmed = True
                    target_id = tid

                    # Mở rộng bounding box thêm 20%
                    w = x2 - x1
                    h = y2 - y1
                    padding = 0.2
                    x1_padded = max(0, int(x1 - w * padding))
                    y1_padded = max(0, int(y1 - h * padding))
                    x2_padded = min(W, int(x2 + w * padding))
                    y2_padded = min(H, int(y2 + h * padding))

                    csrt_tracker = cv2.TrackerCSRT_create()
                    csrt_tracker.init(frame, (x1_padded, y1_padded, x2_padded - x1_padded, y2_padded - y1_padded))
                    print(f"[INFO] Admin re-selected => ID={tid}")
                else:
                    print("[INFO] Person in center, no V Sign => waiting...")

                current_state = STATE_TRACKING

            # Hiển thị frame đã xử lý
            cv2.imshow("Frame", frame)

        # Kiểm tra phím 'Esc' để thoát
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cv2.destroyAllWindows()

############################################
# 6) MAIN
############################################
if __name__ == "__main__":
    # Khởi tạo BodyCenterDetector và VSignDetector
    body_center_detector = BodyCenterDetector()
    vsign_detector = VSignDetector(max_num_hands=1, detection_confidence=0.5, tracking_confidence=0.5)

    try:
        # Start receiver thread
        t_recv = threading.Thread(target=receiver_thread, args=(conn,), daemon=True)
        t_recv.start()

        # Start processing thread và truyền body_center_detector và vsign_detector
        t_proc = threading.Thread(target=processing_thread, args=(body_center_detector, vsign_detector), daemon=True)
        t_proc.start()

        # Vòng lặp chính chờ
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("[INFO] Stopped by user.")
    finally:
        # Đóng các tài nguyên
        body_center_detector.close()
        vsign_detector.close()
        conn.close()
        server.close()
        cv2.destroyAllWindows()
        print("[INFO] Main thread exiting.")
