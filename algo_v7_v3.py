"""
algo_v7_xoay_ne.py - Robot tránh vật cản bằng camera (ESP32-CAM + ESP8266 + L298N)

Phương pháp: người dùng CLICK vào vật cản trên video -> chương trình lưu dải màu HSV của vật
-> mọi pixel nằm trong dải màu đó là "vật cản" -> đo mật độ ở 3 vùng trái/giữa/phải
-> máy trạng thái quyết định đi thẳng / né / lùi.

Cập nhật từ algo_v6:
  - Giảm tốc độ và độ nhạy lái; có vùng chết chống tự lượn.
  - TURN_L/TURN_R xoay tại chỗ (một bánh tiến, một bánh lùi).
  - FORWARD đi thẳng đều hai bánh; cả 3 vùng bị chắn thì lùi rồi xoay về phía ít vật cản.
  - FLIP_HORIZONTAL: bật/tắt lật ngang ảnh (trái <-> phải) bằng một biến.

Firmware: dk_dong_co_wifi_v3.ino (có /drive + watchdog) và code_camv2.ino.
Phím tắt: R = chọn lại mẫu.
"""
import csv
import os
import threading
import time

import customtkinter as ctk
import cv2
import numpy as np
import requests
from PIL import Image

# ===================== 1. CẤU HÌNH IP =====================
# Sửa cho khớp IP in ra ở Serial Monitor của từng board
CAM_URL = "http://10.64.131.200/stream"
ROBOT_IP = "http://10.64.131.201/"

# Lật ngang ảnh (trái <-> phải). Bật nếu camera cho ảnh bị ngược trái/phải so với xe.
FLIP_HORIZONTAL = True

# ===================== 2. NGƯỠNG & LOGIC NÉ =====================
ENTER = 0.35          # vùng GIỮA vượt ngưỡng này -> bắt đầu né
EXIT = 0.22           # vùng GIỮA xuống dưới ngưỡng này -> hết né (trễ/hysteresis)
ALPHA = 0.4           # làm mượt mật độ: d = ALPHA*mới + (1-ALPHA)*cũ
TURN_TIMEOUT = 0.8    # tối đa 1 giây xoay, sau đó lùi nếu vẫn chắn
BACK_TIME = 0.50      # lùi ngắn rồi quay về hướng ít vật cản
ROI_FRAC = 0.6        # chỉ xét 60% phía dưới ảnh (vùng gần chân robot)

# ===================== 3. TỐC ĐỘ (0-255, khớp analogWriteRange(255)) =====================
BASE_SPEED = 80
TURN_SPEED = 90
BACK_SPEED = 90
MIN_PWM = 60          # thử trên xe thật: nếu motor ù mà không chạy, tăng dần
STEER_GAIN = 0.35     # độ gắt khi chỉnh lái nhẹ
SLOW_GAIN = 0.7       # giảm tốc yêu cầu khi có vật cản giữa (chịu giới hạn MIN_PWM)
STEER_DEADBAND = 0.08 # bỏ qua lệch mật độ trái/phải nhỏ

# Cân chỉnh sau khi thử đường thẳng: nếu xe lệch TRÁI, giảm RIGHT_TRIM;
# nếu xe lệch PHẢI, giảm LEFT_TRIM. Để 1.0 trước khi hiệu chuẩn.
LEFT_TRIM = 1.00
RIGHT_TRIM = 1.00

# Xoay tại chỗ: một bánh tiến (+), bánh kia lùi (-).
# TURN_SPEED có thể hạ từ 95 về 80 nếu xoay quá mạnh.
TURN_MIN_TIME = 0.2   # tránh thoát trạng thái né ngay do nhiễu một khung hình

# ===================== 4. MÀU VẬT CẢN (thang HSV của OpenCV: H 0-179, S/V 0-255) =====================
TOL_H, TOL_S, TOL_V = 12, 60, 70

# ===================== 5. GIAO TIẾP & HIỂN THỊ =====================
FRAME_TIMEOUT = 1.0   # quá 1s không có khung hình mới -> coi như mất camera
HEARTBEAT = 0.5       # gửi lại lệnh ít nhất mỗi 0.5s
MIN_INTERVAL = 0.1    # hai lệnh khác nhau cách nhau tối thiểu 0.1s
DISP_W, DISP_H = 640, 480

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


def odd(n, minimum=3):
    """Làm tròn n thành số lẻ >= minimum (kích thước kernel)."""
    n = max(minimum, int(round(n)))
    return n if n % 2 == 1 else n + 1


# =====================================================================
# BỘ ĐIỀU KHIỂN NÉ (thuần Python, không phụ thuộc GUI -> dễ test)
# =====================================================================
class Avoider:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = "FORWARD"
        self.d = None            # mật độ đã làm mượt [trái, giữa, phải]
        self.t_state = 0.0

    def _go(self, state, now):
        self.state = state
        self.t_state = now

    def _smooth(self, raw):
        if self.d is None:
            self.d = list(raw)
        else:
            self.d = [ALPHA * r + (1 - ALPHA) * o for r, o in zip(raw, self.d)]
        return self.d

    @staticmethod
    def _min_pwm(v):
        if abs(v) < 1:
            return 0
        sign = 1 if v > 0 else -1
        return int(sign * min(255, max(MIN_PWM, abs(v))))

    def update(self, raw, now):
        """raw=(trái, giữa, phải), mật độ 0..1. Trả về (state, PWM trái, PWM phải)."""
        l, c, r = self._smooth(raw)
        all_blocked = l >= ENTER and c >= ENTER and r >= ENTER

        if self.state == "FORWARD":
            if all_blocked:
                # Không có hướng thoáng: lùi trước rồi chọn bên ít vật hơn.
                self._go("BACK", now)
            elif c >= ENTER:
                # Giữa bị chắn: ưu tiên bên có ít vật hơn.
                self._go("TURN_L" if l <= r else "TURN_R", now)

        elif self.state in ("TURN_L", "TURN_R"):
            elapsed = now - self.t_state
            if all_blocked and elapsed >= TURN_MIN_TIME:
                self._go("BACK", now)
            elif elapsed >= TURN_MIN_TIME and c < EXIT:
                # Hết vật cản giữa: NGỪNG XOAY, trở lại đi thẳng đều 2 bánh.
                self._go("FORWARD", now)
            elif elapsed >= TURN_TIMEOUT:
                self._go("BACK", now)

        elif self.state == "BACK":
            if now - self.t_state >= BACK_TIME:
                # Sau khi lùi, chọn hướng có mật độ vật cản thấp hơn.
                # Nếu bằng nhau thì mặc định trái.
                self._go("TURN_L" if l <= r else "TURN_R", now)

        return (self.state,) + self._speeds(l, c, r)

    def _speeds(self, l, c, r):
        if self.state == "TURN_L":
            return -TURN_SPEED, TURN_SPEED
        if self.state == "TURN_R":
            return TURN_SPEED, -TURN_SPEED
        if self.state == "BACK":
            return -BACK_SPEED, -BACK_SPEED
        # FORWARD: CẢ HAI BÁNH CÙNG PWM, không bẻ lái theo mật độ trái/phải.
        # Bỏ steer vì nó khiến xe tiếp tục đi cong sau khi đã né xong.
        # Trim (nếu có) vẫn được áp dụng ở RobotLink.drive để bù sai lệch motor.
        slow = 1.0 - SLOW_GAIN * min(c / ENTER, 1.0)
        speed = self._min_pwm(BASE_SPEED * slow)
        return speed, speed


# =====================================================================
# GỬI LỆNH TỚI ROBOT (1 thread duy nhất, heartbeat)
# =====================================================================
class RobotLink:
    def __init__(self, base_url):
        self.base = base_url.rstrip("/")
        self.session = requests.Session()
        self.path = "stop"
        self.online = False
        self._last_path = None
        self._last_sent = 0.0
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def set(self, path):
        with self._lock:
            self.path = path

    def drive(self, left, right):
        # Trim đặt ở bước cuối, áp dụng cả tiến/lùi/cua.
        # Làm tròn theo bước 5 như phiên bản cũ để tránh gửi quá nhiều lệnh.
        def q(v):
            v = max(-255, min(255, v))
            return int(round(v / 5.0) * 5)

        self.set(f"drive?l={q(left * LEFT_TRIM)}&r={q(right * RIGHT_TRIM)}")

    def _loop(self):
        while self._running:
            with self._lock:
                path = self.path
            now = time.time()
            due = now - self._last_sent >= HEARTBEAT
            changed = path != self._last_path and now - self._last_sent >= MIN_INTERVAL
            if due or changed:
                self._send(path)
            time.sleep(0.02)

    def _send(self, path):
        try:
            self.session.get(f"{self.base}/{path}", timeout=0.6)
            self.online = True
        except requests.RequestException:
            self.online = False
        self._last_path = path
        self._last_sent = time.time()

    def shutdown(self):
        self._running = False
        try:
            requests.get(f"{self.base}/stop", timeout=0.5)
        except requests.RequestException:
            pass


# =====================================================================
# NHẬN VIDEO TỪ ESP32-CAM
# =====================================================================
class VideoStream:
    def __init__(self, src):
        self.src = src
        self.stream = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frame = None
        self.ts = 0.0
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        while not self.stopped:
            if not self.stream.isOpened():
                self.stream.open(self.src, cv2.CAP_FFMPEG)
                if not self.stream.isOpened():
                    time.sleep(1)
                    continue
            grabbed, frame = self.stream.read()
            if grabbed:
                if FLIP_HORIZONTAL:
                    frame = cv2.flip(frame, 1)      # đảo trái <-> phải
                with self.lock:
                    self.frame = frame
                    self.ts = time.time()
            else:
                self.stream.release()
                time.sleep(1)

    def read(self):
        with self.lock:
            return self.frame, self.ts

    def stop(self):
        self.stopped = True
        self.stream.release()


# =====================================================================
# ỨNG DỤNG GUI
# =====================================================================
class RobotControllerApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("ROBOT OBSTACLE AVOIDANCE v7 - CLICK VẬT CẢN")
        self.geometry("1120x720")
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- trạng thái phát hiện ---
        self.samples = []                 # các mẫu HSV đã click
        self.hsv_lo = self.hsv_hi = None
        self.last_blur = None
        self.view_mode = "Real"
        self.cam_connected = False
        self._last_status = 0.0

        # --- log ---
        self.log_file = None
        self.log_writer = None
        self.log_t0 = 0.0

        self.avoider = Avoider()
        self.link = RobotLink(ROBOT_IP)

        self.setup_ui()
        for key in ("r", "R"):
            self.bind(f"<{key}>", lambda e: self.reset_calibration())

        self.vs = VideoStream(CAM_URL).start()
        self.update_gui()

    # ---------------- UI ----------------
    def setup_ui(self):
        self.sidebar = ctk.CTkFrame(self, width=290, corner_radius=0)
        self.sidebar.grid(row=0, column=0, sticky="nsew")
        self.sidebar.grid_propagate(False)

        def title(text):
            ctk.CTkLabel(self.sidebar, text=text, font=ctk.CTkFont(size=16, weight="bold")).pack(
                padx=20, pady=(14, 4), anchor="w")

        title("TRẠNG THÁI HỆ THỐNG")
        status = ctk.CTkFrame(self.sidebar, fg_color="#2b2b2b")
        status.pack(padx=20, pady=4, fill="x")
        self.lbl_cam_status = ctk.CTkLabel(status, text="● CAM: CHECKING...", font=("Arial", 12, "bold"), text_color="gray")
        self.lbl_cam_status.pack(pady=3, padx=10, anchor="w")
        self.lbl_robot_status = ctk.CTkLabel(status, text="● ESP: CHECKING...", font=("Arial", 12, "bold"), text_color="gray")
        self.lbl_robot_status.pack(pady=3, padx=10, anchor="w")
        self.lbl_lat = ctk.CTkLabel(status, text="Trễ xử lý: -- ms", font=("Arial", 12), text_color="gray")
        self.lbl_lat.pack(pady=3, padx=10, anchor="w")

        title("HƯỚNG DẪN")
        ctk.CTkLabel(self.sidebar,
                     text="Click vào vật cản trên video. Click thêm nhiều điểm (cả vùng sáng/tối của vật) "
                          "để mở rộng dải màu. Bật 'Xem Mask' để kiểm tra: vật cản trắng, nền đen.",
                     wraplength=250, justify="left", font=("Arial", 11), text_color="#bdc3c7").pack(
            padx=20, pady=(0, 4), anchor="w")

        title("ĐIỀU KHIỂN")
        self.btn_reset = ctk.CTkButton(self.sidebar, text="CHỌN LẠI MẪU (R)", command=self.reset_calibration,
                                       fg_color="#c0392b", hover_color="#e74c3c")
        self.btn_reset.pack(padx=20, pady=4, fill="x")
        self.switch_view = ctk.CTkSwitch(self.sidebar, text="Xem Mask Vật Cản", command=self.toggle_view)
        self.switch_view.pack(padx=20, pady=6, anchor="w")
        self.switch_log = ctk.CTkSwitch(self.sidebar, text="Ghi log CSV", command=self.toggle_log)
        self.switch_log.pack(padx=20, pady=6, anchor="w")

        title("MỨC ĐỘ NGUY HIỂM")
        self.bar_left = self.create_bar("TRÁI")
        self.bar_center = self.create_bar("GIỮA")
        self.bar_right = self.create_bar("PHẢI")

        ctk.CTkLabel(self.sidebar, text="HÀNH ĐỘNG:", font=ctk.CTkFont(size=14)).pack(padx=20, pady=(12, 0), anchor="w")
        self.lbl_cmd = ctk.CTkLabel(self.sidebar, text="CHỜ...", font=ctk.CTkFont(size=22, weight="bold"), text_color="#f1c40f")
        self.lbl_cmd.pack(padx=20, pady=(0, 10), anchor="w")

        # khung video (label kích thước cố định, căn giữa)
        self.video_frame = ctk.CTkFrame(self, corner_radius=10)
        self.video_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.lbl_video = ctk.CTkLabel(self.video_frame, text="ĐANG KẾT NỐI...", font=ctk.CTkFont(size=20),
                                      width=DISP_W + 20, height=DISP_H + 20)
        self.lbl_video.pack(expand=True, padx=10, pady=10)
        self.lbl_video.bind("<Button-1>", self.on_mouse_click)

    def create_bar(self, name):
        ctk.CTkLabel(self.sidebar, text=name, anchor="w", font=ctk.CTkFont(size=12)).pack(padx=20, pady=(4, 0), anchor="w")
        bar = ctk.CTkProgressBar(self.sidebar, orientation="horizontal")
        bar.pack(padx=20, pady=(0, 2), fill="x")
        bar.set(0)
        bar.configure(progress_color="#2ecc71")
        return bar

    def toggle_view(self):
        self.view_mode = "Mask" if self.switch_view.get() == 1 else "Real"

    # ---------------- LOG ----------------
    def toggle_log(self):
        if self.switch_log.get() == 1:
            os.makedirs("logs", exist_ok=True)
            name = time.strftime("logs/run_%Y%m%d_%H%M%S.csv")
            self.log_file = open(name, "w", newline="", encoding="utf-8")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(["t_s", "d_left", "d_center", "d_right",
                                      "state", "cmd_l", "cmd_r", "latency_ms"])
            self.log_t0 = time.time()
            print("Dang ghi log:", name)
        else:
            self.close_log()

    def close_log(self):
        if self.log_file:
            self.log_file.close()
        self.log_file = None
        self.log_writer = None

    # ---------------- LẤY MẪU MÀU VẬT CẢN ----------------
    def _scaling(self):
        try:
            return self.lbl_video._get_widget_scaling()
        except Exception:
            return 1.0

    def on_mouse_click(self, event):
        if self.last_blur is None:
            return
        s = self._scaling()
        lw, lh = self.lbl_video.winfo_width(), self.lbl_video.winfo_height()
        ox, oy = (lw - DISP_W * s) / 2.0, (lh - DISP_H * s) / 2.0
        x = (event.x_root - self.lbl_video.winfo_rootx() - ox) / s
        y = (event.y_root - self.lbl_video.winfo_rooty() - oy) / s
        if not (0 <= x < DISP_W and 0 <= y < DISP_H):
            return
        h, w = self.last_blur.shape[:2]
        px, py = int(x * w / DISP_W), int(y * h / DISP_H)
        r = 1
        patch = self.last_blur[max(0, py - r):py + r + 1, max(0, px - r):px + r + 1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        self.samples.append(np.median(hsv, axis=0))
        self.rebuild_color_range()
        print(f"Mau #{len(self.samples)} HSV={self.samples[-1]} -> lo={self.hsv_lo} hi={self.hsv_hi}")

    def rebuild_color_range(self):
        S = np.array(self.samples, dtype=np.float32)
        h0 = S[0, 0]
        dh = ((S[:, 0] - h0 + 90) % 180) - 90          # lệch hue so với mẫu đầu, có xử lý vòng
        h_lo = int(round(h0 + dh.min() - TOL_H))
        h_hi = int(round(h0 + dh.max() + TOL_H))
        s_lo = int(max(0, S[:, 1].min() - TOL_S))
        s_hi = int(min(255, S[:, 1].max() + TOL_S))
        v_lo = int(max(0, S[:, 2].min() - TOL_V))
        v_hi = int(min(255, S[:, 2].max() + TOL_V))
        self.hsv_lo = (h_lo, s_lo, v_lo)
        self.hsv_hi = (h_hi, s_hi, v_hi)

    def reset_calibration(self):
        self.samples = []
        self.hsv_lo = self.hsv_hi = None
        self.avoider.reset()
        self.link.set("stop")
        self.set_cmd("DỪNG (RESET)", "#95a5a6")

    # ---------------- MASK ----------------
    def make_mask(self, blurred):
        """Mask trắng = VẬT CẢN (pixel nằm trong dải màu đã click)."""
        if not self.samples:
            return None
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        (h_lo, s_lo, v_lo), (h_hi, s_hi, v_hi) = self.hsv_lo, self.hsv_hi
        rng = lambda a, b: cv2.inRange(hsv, (a, s_lo, v_lo), (b, s_hi, v_hi))
        if h_hi - h_lo >= 179:
            return rng(0, 179)
        if h_lo < 0:                                    # dải hue vòng qua 0 (màu đỏ)
            return cv2.bitwise_or(rng(0, h_hi), rng(180 + h_lo, 179))
        if h_hi > 179:
            return cv2.bitwise_or(rng(h_lo, 179), rng(0, h_hi - 180))
        return rng(h_lo, h_hi)

    # ---------------- VÒNG LẶP CHÍNH ----------------
    def update_gui(self):
        frame, ts = self.vs.read()
        now = time.time()
        self.cam_connected = frame is not None and (now - ts) < FRAME_TIMEOUT

        if self.cam_connected:
            self.process(frame, ts)
        else:
            # mất camera -> dừng xe ngay
            self.link.set("stop")
            self.avoider.reset()
            self.set_cmd("MẤT CAMERA - DỪNG", "#7f8c8d")
            self.lbl_video.configure(text="MẤT CAMERA...", image=None)

        if now - self._last_status > 0.5:
            self._last_status = now
            self.update_connection_ui()
        self.after(30, self.update_gui)

    def process(self, frame, ts):
        h, w = frame.shape[:2]
        bk = odd(5 * w / 320.0)
        blurred = cv2.GaussianBlur(frame, (bk, bk), 0)
        self.last_blur = blurred

        mask = self.make_mask(blurred)
        dens = (0.0, 0.0, 0.0)

        if mask is None:
            # chưa click vật cản -> đứng yên
            self.link.set("stop")
            self.avoider.reset()
            self.set_cmd("CLICK VÀO VẬT CẢN", "#95a5a6")
        else:
            # lọc nhiễu (kernel co giãn theo độ phân giải)
            k = odd(3 * w / 320.0)
            kernel = np.ones((k, k), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # ROI = phần dưới ảnh, chia 3 vùng
            roi_h = int(h * ROI_FRAC)
            roi = mask[h - roi_h:h, :]
            edges = [0, w // 3, 2 * w // 3, w]
            raw = []
            for i in range(3):
                part = roi[:, edges[i]:edges[i + 1]]
                raw.append(cv2.countNonZero(part) / float(part.size))

            state, cl, cr = self.avoider.update(raw, time.time())
            dens = tuple(self.avoider.d)

            if state == "FORWARD":
                self.set_cmd("ĐI THẲNG ^", "#2ecc71")
            elif state == "TURN_L":
                self.set_cmd("<< NÉ SANG TRÁI", "#e74c3c")
            elif state == "TURN_R":
                self.set_cmd("NÉ SANG PHẢI >>", "#e74c3c")
            else:
                self.set_cmd("TẮC ĐƯỜNG (LÙI)", "#c0392b")
            self.link.drive(cl, cr)

            if not self.link.online:
                self.set_cmd("MẤT KẾT NỐI ROBOT", "#7f8c8d")

            lat = (time.time() - ts) * 1000.0
            self.lbl_lat.configure(text=f"Trễ xử lý: {lat:.0f} ms")
            if self.log_writer:
                self.log_writer.writerow([f"{time.time() - self.log_t0:.3f}",
                                          f"{dens[0]:.3f}", f"{dens[1]:.3f}", f"{dens[2]:.3f}",
                                          state, cl, cr, f"{lat:.0f}"])

        self.show_bars(dens)
        self.show_frame(frame, mask, dens)

    # ---------------- HIỂN THỊ ----------------
    def show_bars(self, dens):
        for bar, v in zip((self.bar_left, self.bar_center, self.bar_right), dens):
            bar.set(min(1.0, v))
            bar.configure(progress_color="#e74c3c" if v > ENTER else "#2ecc71")

    def show_frame(self, frame, mask, dens):
        disp = cv2.resize(frame, (DISP_W, DISP_H), interpolation=cv2.INTER_LINEAR)
        top = DISP_H - int(DISP_H * ROI_FRAC)
        xs = [0, DISP_W // 3, 2 * DISP_W // 3, DISP_W]

        if self.view_mode == "Mask" and mask is not None:
            m = cv2.resize(mask, (DISP_W, DISP_H), interpolation=cv2.INTER_NEAREST)
            m[:top, :] = 0
            disp = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)

        if mask is not None:
            for i in range(3):
                if dens[i] > ENTER:
                    cv2.rectangle(disp, (xs[i], top), (xs[i + 1], DISP_H), (0, 0, 255), 3)
                    cv2.putText(disp, "VAT CAN", (xs[i] + 10, top + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.rectangle(disp, (xs[i], top), (xs[i + 1], DISP_H), (0, 255, 0), 1)

        img = Image.fromarray(cv2.cvtColor(disp, cv2.COLOR_BGR2RGB))
        self._ctk_img = ctk.CTkImage(light_image=img, dark_image=img, size=(DISP_W, DISP_H))
        self.lbl_video.configure(text="", image=self._ctk_img)

    def set_cmd(self, text, color):
        self.lbl_cmd.configure(text=text, text_color=color)

    def update_connection_ui(self):
        if self.link.online:
            self.lbl_robot_status.configure(text="● ESP: ONLINE", text_color="#2ecc71")
        else:
            self.lbl_robot_status.configure(text="● ESP: OFFLINE", text_color="#e74c3c")
        if self.cam_connected:
            self.lbl_cam_status.configure(text="● CAM: ONLINE", text_color="#2ecc71")
        else:
            self.lbl_cam_status.configure(text="● CAM: OFFLINE", text_color="#e74c3c")

    def on_close(self):
        self.close_log()
        self.vs.stop()
        self.link.shutdown()      # gửi STOP trước khi thoát
        self.destroy()


if __name__ == "__main__":
    app = RobotControllerApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
