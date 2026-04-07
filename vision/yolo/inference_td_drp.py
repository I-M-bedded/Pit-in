import os
import glob
import torch
from ultralytics import YOLO
import sys

# 프로젝트 루트 경로 추가 (TD-DRP 모듈 임포트용)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from vision.yolo.train_td_drp_pose import TDDRPEndToEndPose

# ==========================================
# [설정 영역]
# ==========================================
# 우리가 방금 학습시킨 TD-DRP 최고 성능 가중치
TD_DRP_WEIGHTS = os.path.abspath("result/td_drp_yolo26n_pose/td_drp_phase2_best.pt")
# 베이스가 되는 YOLO 모델 가중치 (구조 로드용)
BASE_YOLO_WEIGHTS = os.path.abspath("runs/pose/runs/train/yolo26n-pose/weights/best.pt")

# 타겟 이미지들이 있는 경로 (output_masks_9 포함)
TEST_IMG_DIR = os.path.abspath("vision/dataset/yolo_pose_dataset/test/images")
# 결과 저장 경로
SAVE_DIR = os.path.abspath("result/td_drp_yolo26n_pose/test_inference_output_masks_9")
# ==========================================

def main():
    if not os.path.exists(TD_DRP_WEIGHTS):
        print(f"에러: TD-DRP 학습 가중치를 찾을 수 없습니다 -> {TD_DRP_WEIGHTS}")
        return

    os.makedirs(SAVE_DIR, exist_ok=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    print(f"🚀 TD-DRP 모델 로딩 중... ({TD_DRP_WEIGHTS})")
    
    # 1. TD-DRP 커스텀 모델 구조 생성 및 가중치 적재
    td_drp_model = TDDRPEndToEndPose(BASE_YOLO_WEIGHTS)
    td_drp_model.load_state_dict(torch.load(TD_DRP_WEIGHTS, map_location=device))
    td_drp_model.to(device)
    td_drp_model.eval()

    # 2. 결과 시각화(Annotator) 및 NMS 후처리를 날먹(?)하기 위해 Ultralytics 엔진 로드
    print(f"📦 Ultralytics 추론 엔진 로딩 중...")
    yolo = YOLO(BASE_YOLO_WEIGHTS)
    
    # 3. 몽키 패칭(Monkey Patching) ⭐️
    # YOLO의 내부 추론(forward) 로직을 가로채서, 일반 YOLO가 아닌 '조명 보정 모듈이 결합된 TD-DRP'를 거치게 만듭니다.
    def custom_forward(x, *args, **kwargs):
        # x는 입력 이미지 텐서 (B, 3, H, W)
        with torch.no_grad():
            outputs = td_drp_model(x)
        # Ultralytics 엔진은 원래 튜플 형태의 출력을 기대하므로, pose_logits를 반환
        return outputs["pose_logits"]

    # 원래의 forward 함수를 우리가 만든 custom_forward로 바꿔치기
    yolo.model.forward = custom_forward
    
    # 4. 타겟 이미지 검색 (output_masks_9 가 포함된 이미지)
    img_paths = glob.glob(os.path.join(TEST_IMG_DIR, "*output_masks_9*.png")) + \
                glob.glob(os.path.join(TEST_IMG_DIR, "*output_masks_9*.jpg"))
    
    if not img_paths:
        print(f"경고: {TEST_IMG_DIR} 에서 'output_masks_9' 가 포함된 이미지를 찾을 수 없습니다.")
        return
        
    print(f"\n✅ 타겟 이미지 {len(img_paths)}장을 찾았습니다. 추론을 시작합니다...")
    
    # 5. 추론 실행 (박스와 라벨은 숨기고 깨끗한 마스크/키포인트만 출력)
    yolo.predict(
        source=img_paths,
        save=True,
        imgsz=640,
        conf=0.25,
        show_boxes=False,  # Bounding Box 숨기기
        show_labels=False, # Label 텍스트 숨기기
        project=SAVE_DIR,
        name="predict",
        exist_ok=True
    )
    
    print(f"\n🎉 추론이 완료되었습니다!")
    print(f"결과 이미지들은 [{os.path.join(SAVE_DIR, 'predict')}] 에서 확인하실 수 있습니다.")

if __name__ == "__main__":
    main()
