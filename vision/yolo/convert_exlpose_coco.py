import os
import json
import numpy as np

def convert_coco_to_yolo(coco_json_path, output_dir):
    """COCO JSON 포맷을 YOLO Pose (.txt) 포맷으로 변환합니다."""
    print(f"변환 시작: {coco_json_path}")
    os.makedirs(output_dir, exist_ok=True)
    
    with open(coco_json_path, 'r') as f:
        data = json.load(f)
        
    # 이미지 ID로 매핑
    img_dict = {img['id']: img for img in data['images']}
    
    # 변환된 라벨 저장 딕셔너리
    yolo_labels = {}
    
    for ann in data['annotations']:
        img_id = ann['image_id']
        img_info = img_dict[img_id]
        img_w, img_h = img_info['width'], img_info['height']
        filename = os.path.basename(img_info['file_name']) # 'dark/xxx.png' -> 'xxx.png'
        
        # Bounding Box: [x_min, y_min, width, height] -> YOLO [cx, cy, w, h] (normalized)
        bbox = ann['bbox']
        x_min, y_min, w, h = bbox
        cx = (x_min + w / 2) / img_w
        cy = (y_min + h / 2) / img_h
        nw = w / img_w
        nh = h / img_h
        
        # 클리핑 (0~1)
        cx, cy, nw, nh = map(lambda x: np.clip(x, 0.0, 1.0), [cx, cy, nw, nh])
        
        # Keypoints: [x1, y1, v1, x2, y2, v2, ...] -> normalized
        kpts = ann['keypoints']
        norm_kpts = []
        for i in range(0, len(kpts), 3):
            kx = np.clip(kpts[i] / img_w, 0.0, 1.0)
            ky = np.clip(kpts[i+1] / img_h, 0.0, 1.0)
            vis = kpts[i+2]
            
            # YOLO visibility: 0=안보임/화면밖, 1=가려짐(occluded), 2=보임
            # COCO visibility: 0=없음, 1=가려짐, 2=보임
            # 거의 1:1 매칭이지만 확실히 하기 위해
            if vis > 0 and (kpts[i] < 0 or kpts[i] >= img_w or kpts[i+1] < 0 or kpts[i+1] >= img_h):
                vis = 0 # 화면 밖
                
            norm_kpts.extend([kx, ky, vis])
            
        # Class ID: 0 (YOLO에서는 'person'이 보통 0, ExLPose는 category_id가 1부터 시작하므로 -1)
        class_id = 0 
        
        line = f"{class_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f} " + " ".join([f"{k:.6f}" if i%3 != 2 else f"{int(k)}" for i, k in enumerate(norm_kpts)])
        
        txt_filename = os.path.splitext(filename)[0] + ".txt"
        if txt_filename not in yolo_labels:
            yolo_labels[txt_filename] = []
        yolo_labels[txt_filename].append(line)
        
    # 파일 쓰기
    count = 0
    for txt_filename, lines in yolo_labels.items():
        txt_path = os.path.join(output_dir, txt_filename)
        with open(txt_path, 'w') as f:
            f.write("\n".join(lines))
        count += 1
        
    print(f"✅ {count}개의 YOLO 형식 텍스트 라벨이 {output_dir} 에 저장되었습니다.")
    
# 실행: ExLPose_train_LL 변환
base_dir = "vision/dataset/ExLPose"
train_json = os.path.join(base_dir, "Annotations/Annotations/ExLPose/ExLPose_train_LL.json")
train_labels_dir = os.path.join(base_dir, "ExLPose/dark/labels") # dark(Low-Light) 폴더 옆에 labels 생성

convert_coco_to_yolo(train_json, train_labels_dir)
