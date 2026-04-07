import os
import shutil
import glob
from pathlib import Path
from ultralytics import YOLO

# ==========================================
# [사용자 설정 영역]
# ==========================================
# 학습할 데이터셋 YAML 파일 경로
SEG_DATASET_YAML = os.path.abspath("vision/dataset/yolo_dataset/dataset.yaml")
POSE_DATASET_YAML = os.path.abspath("vision/dataset/yolo_pose_dataset/dataset.yaml")

# 비교할 모델 리스트 (v8, v11, v26의 pose 및 seg)
MODELS_TO_TRAIN = [
    "yolov8n-seg.pt",
    "yolov8n-pose.pt",
    "yolo11n-seg.pt",
    "yolo11n-pose.pt",
    "yolo26n-seg.pt",
    "yolo26n-pose.pt"
]

# 공통 학습 하이퍼파라미터
EPOCHS = 100
IMG_SIZE = 640
BATCH_SIZE = 64  # RTX 4070 (12GB VRAM) 활용을 위해 16 -> 64로 상향
WORKERS = 8      # 데이터 로딩 병목 방지를 위한 워커 수 설정

# 결과를 저장할 최상위 디렉토리
RESULT_BASE_DIR = os.path.abspath("result")
# ==========================================

def copy_plots_and_metrics(train_run_dir: str, save_dir: str):
    """학습 결과 디렉토리에서 주요 플롯(Loss, mAP 등)을 복사합니다."""
    os.makedirs(save_dir, exist_ok=True)
    
    # Ultralytics가 기본적으로 생성하는 플롯 파일들
    target_files = [
        "results.png",          # Loss 및 Metrics 추이 그래프
        "confusion_matrix.png", # Confusion Matrix
        "val_batch0_labels.jpg",
        "val_batch0_pred.jpg"
    ]
    
    for file_name in target_files:
        src_path = os.path.join(train_run_dir, file_name)
        if os.path.exists(src_path):
            shutil.copy(src_path, os.path.join(save_dir, file_name))
            
    # metrics 요약 파일 (csv)
    results_csv = os.path.join(train_run_dir, "results.csv")
    if os.path.exists(results_csv):
        shutil.copy(results_csv, os.path.join(save_dir, "results.csv"))

def run_test_inference(model: YOLO, model_name: str, test_images_dir: str, save_dir: str):
    """Test 데이터셋에 대해 추론을 수행하고 결과를 저장합니다."""
    print(f"\n[{model_name}] Test 셋 추론을 시작합니다...")
    
    inference_save_dir = os.path.join(save_dir, "test_inference")
    os.makedirs(inference_save_dir, exist_ok=True)
    
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
        name="test_inference",
        exist_ok=True # 덮어쓰기 허용
    )
    print(f"[{model_name}] Test 추론 완료. 저장 위치: {os.path.join(save_dir, 'test_inference')}")

def main():
    os.makedirs(RESULT_BASE_DIR, exist_ok=True)

    print("==================================================")
    print("🚀 YOLO 다중 모델 학습 및 평가 자동화 스크립트 시작")
    print("==================================================")

    for model_name in MODELS_TO_TRAIN:
        print(f"\n\n{'='*50}")
        print(f" 🟢 모델 시작: {model_name}")
        print(f"{'='*50}")
        
        # 모델 이름에서 확장자 제거 (예: yolov8n-seg)
        model_basename = os.path.splitext(model_name)[0]
        
        # 결과를 저장할 개별 폴더
        model_result_dir = os.path.join(RESULT_BASE_DIR, model_basename)
        os.makedirs(model_result_dir, exist_ok=True)
        
        # 모델에 맞는 데이터셋 경로 선택
        if "pose" in model_name.lower():
            current_dataset_yaml = POSE_DATASET_YAML
        else:
            current_dataset_yaml = SEG_DATASET_YAML

        if not os.path.exists(current_dataset_yaml):
            print(f"에러: 모델 {model_name}에 필요한 데이터셋 YAML 파일을 찾을 수 없습니다 -> {current_dataset_yaml}")
            print("데이터셋 변환(convert_seg_to_pose.py)을 수행했는지 확인해주세요.")
            continue

        dataset_root = os.path.dirname(current_dataset_yaml)
        test_images_dir = os.path.join(dataset_root, "test", "images")
        
        # 1. 모델 로드
        model = YOLO(model_name)
        
        # 2. 모델 학습
        print(f"[{model_name}] 학습을 시작합니다... (Epochs: {EPOCHS})")
        # 학습 결과는 model_result_dir/train_run 에 저장됨
        train_results = model.train(
            data=current_dataset_yaml,
            epochs=EPOCHS,
            imgsz=IMG_SIZE,
            batch=BATCH_SIZE,
            workers=WORKERS, # 데이터 로딩 병렬화 추가
            rect=True,       # ⭐️ 핵심: 종횡비(16:9 등)를 유지하고 남는 공간은 패딩(Letterbox) 처리
            project=model_result_dir,
            name="train_run",
            exist_ok=True, # 기존 학습 내역 덮어쓰기
            device='0' # GPU 번호 (여러 개면 '0,1' 식으로 변경)
        )
        
        # 3. 학습 결과(Loss, mAP 그래프 등) 결과 폴더로 복사
        train_run_dir = os.path.join(model_result_dir, "train_run")
        print(f"[{model_name}] 학습 완료. 플롯 이미지를 결과 폴더로 복사합니다.")
        copy_plots_and_metrics(train_run_dir, model_result_dir)
        
        # 4. 베스트 모델 로드 후 Test 셋 추론
        best_weights = os.path.join(train_run_dir, "weights", "best.pt")
        if os.path.exists(best_weights):
            best_model = YOLO(best_weights)
            # Test 셋 추론 수행
            run_test_inference(best_model, model_name, test_images_dir, model_result_dir)
            
            # 베스트 가중치 파일도 result 폴더에 백업
            shutil.copy(best_weights, os.path.join(model_result_dir, f"{model_basename}_best.pt"))
        else:
            print(f"경고: {best_weights} 를 찾을 수 없어 추론을 생략합니다.")
            
    print("\n✅ 모든 모델의 학습 및 테스트 과정이 완료되었습니다.")
    print(f"결과는 {RESULT_BASE_DIR} 폴더에서 확인하실 수 있습니다.")

if __name__ == "__main__":
    main()
