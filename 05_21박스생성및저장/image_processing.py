import threading
import cv2
import numpy as np
import random

def load_inspection_config(file_path):
    image_infos = []
    with open(file_path, "r") as f:
        next(f)  # 헤더 건너뜀
        for line_num, line in enumerate(f, start=2):  # 2는 헤더 다음 줄
            parts = line.strip().split(",")

            if len(parts) != 5:
                raise ValueError(f"❌ {file_path}:{line_num} - 필드 개수 오류 (필수 5개, 현재 {len(parts)}개): {line.strip()}")

            name = parts[0].strip()
            path = parts[1].strip()
            distance_m = float(parts[2].strip())
            length_pix = int(parts[3].strip())
            total_length = float(parts[4].strip())

            image_infos.append([name, path, distance_m, length_pix, total_length])

    return image_infos



def calculate_required_pixels(distance_covered_m, pixels_per_meter):
    """
    주어진 거리(m)를 커버하기 위한 총 픽셀 수 계산
    """
    return int(round(distance_covered_m * pixels_per_meter))



def generate_white_image_cv(width=6000, height=4096, save_path="white_image.jpg"):
    # 0 ~ 255 범위의 밝기 값 선택
    brightness = random.randint(0, 255)
    # 모든 픽셀이 255 (흰색)인 BGR 이미지 생성
    white_image = np.ones((height, width, 3), dtype=np.uint8) * brightness
    cv2.imwrite(save_path, white_image)
    print(f"[✅ OpenCV] 저장됨: {save_path}")



class ImageLoadRequestThread(threading.Thread):
    def __init__(self, idx, image_path, callback):
        super().__init__()
        self.idx = idx
        self.image_path = image_path
        self.callback = callback  # 예: def handle_loaded_image(index, image)

    def run(self):
        try:
            image = cv2.imread(self.image_path)
            if image is None:
                raise FileNotFoundError(f"이미지를 불러올 수 없습니다: {self.image_path}")
            self.callback(self.idx, image)
        except Exception as e:
            print(f"[❌ 이미지 로딩 오류] {self.image_path} - {e}")