import os
import glob
from ultralytics import YOLO

# ==========================================
# [사용자 설정 영역]
# ==========================================
# 모델 결과가 저장되어 있는 최상위 디렉토리
RESULT_BASE_DIR = os.path.abspath("result")

# 테스트 이미지가 있는 데이터셋 경로
SEG_TEST_IMAGES = os.path.abspath("vision/dataset/yolo_dataset/test/images")
POSE_TEST_IMAGES = os.path.abspath("vision/dataset/yolo_pose_dataset/test/images")

# 추론할 모델 리스트 (train_compare.py와 동일하게 설정)
MODELS_TO_INFER = [
    "yolov8n-seg.pt",
    "yolov8n-pose.pt",
    "yolo11n-seg.pt",
    "yolo11n-pose.pt",
    "yolo26n-seg.pt",
    "yolo26n-pose.pt"
]

# 추론 시 이미지 사이즈
IMG_SIZE = 640
# ==========================================

def run_test_inference(model: YOLO, model_name: str, test_images_dir: str, save_dir: str):
    """Test 데이터셋에 대해 추론을 수행하고 결과를 저장합니다."""
    print(f"\n[{model_name}] Test 셋 추론을 시작합니다...")
    
    if not os.path.exists(test_images_dir):
        print(f"경고: Test 디렉토리를 찾을 수 없습니다: {test_images_dir}")
        return

    # test 폴더 내의 이미지 검색
    test_imgs = glob.glob(os.path.join(test_images_dir, "*.jpg")) + \
                glob.glob(os.path.join(test_images_dir, "*.png"))
                
    if not test_imgs:
        print("경고: Test 폴더에 이미지가 없습니다.")
        return

    # 추론 수행 및 저장 (save=True 시 자동으로 그려진 이미지가 저장됨)
    model.predict(
        source=test_images_dir,
        save=True,
        imgsz=IMG_SIZE,
        conf=0.25,
        show_boxes=False,  # ⭐️ Bounding Box 숨기기
        show_labels=False, # ⭐️ Label 텍스트(class, conf) 숨기기
        project=save_dir,
        name="test_inference_clean",
        exist_ok=True # 덮어쓰기 허용
    )
    print(f"[{model_name}] Test 추론 완료. 저장 위치: {os.path.join(save_dir, 'test_inference_clean')}")

def main():
    os.makedirs(RESULT_BASE_DIR, exist_ok=True)

    print("==================================================")
    print("🚀 YOLO 다중 모델 깨끗한 추론(Inference) 전용 스크립트 시작")
    print("==================================================")

    for model_name in MODELS_TO_INFER:
        model_basename = os.path.splitext(model_name)[0]
        model_result_dir = os.path.join(RESULT_BASE_DIR, model_basename)
        
        # runs 폴더 구조에서 best 가중치 파일 경로 유추
        if "pose" in model_name.lower():
            best_weights = os.path.abspath(f"runs/pose/runs/train/{model_basename}/weights/best.pt")
        else:
            best_weights = os.path.abspath(f"runs/segment/runs/train/{model_basename}/weights/best.pt")
        
        if not os.path.exists(best_weights):
            print(f"\n[건너뜀] {model_basename} 모델의 학습된 가중치({best_weights})를 찾을 수 없습니다.")
            continue

        print(f"\n\n{'='*50}")
        print(f" 🟢 추론 시작: {model_basename}")
        print(f"{'='*50}")

        # 모델 로드
        print(f"[{model_name}] 저장된 가중치를 불러옵니다: {best_weights}")
        model = YOLO(best_weights)

        # 모델 종류에 따라 테스트 이미지 폴더 선택
        if "pose" in model_name.lower():
            test_images_dir = POSE_TEST_IMAGES
        else:
            test_images_dir = SEG_TEST_IMAGES

        # 추론 실행
        run_test_inference(model, model_basename, test_images_dir, model_result_dir)

    print("\n✅ 모든 모델의 추론 과정이 완료되었습니다.")
    print(f"결과는 {RESULT_BASE_DIR} 폴더 내의 각 모델 폴더 안 'test_inference_clean'에서 확인하실 수 있습니다.")

if __name__ == "__main__":
    main()
