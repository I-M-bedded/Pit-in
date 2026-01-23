#!/bin/bash

NUC_USER="pitin"       
NUC_IP="192.168.55.100"       # NUC의 IP 주소
NUC_PATH="/home/pitin/Desktop/testbed_agv-main" # NUC에 있는 실제 코드 폴더 경로

# 2. 젯슨 설정
LOCAL_MOUNT_POINT="$HOME/NUC_codes"  # 젯슨에 마운트될 경로 (없으면 자동생성)
DOCKER_IMAGE="jetson-vision:v1" # 사용할 도커 이미지

echo "Checking mount status..."

# 마운트 포인트 폴더가 없으면 생성
if [ ! -d "$LOCAL_MOUNT_POINT" ]; then
    echo "Creating mount point at $LOCAL_MOUNT_POINT..."
    mkdir -p "$LOCAL_MOUNT_POINT"
fi

# 이미 마운트되어 있는지 확인 (중복 마운트 방지)
if mountpoint -q "$LOCAL_MOUNT_POINT"; then
    echo "✅ Already mounted."
else
    echo "🔄 Mounting NUC folder via SSHFS..."
    # SSHFS 마운트 시도
    sshfs -o allow_other $NUC_USER@$NUC_IP:$NUC_PATH $LOCAL_MOUNT_POINT
    
    # 마운트 성공 여부 재확인
    if [ $? -eq 0 ]; then
        echo "✅ Mount successful!"
    else
        echo "❌ Mount failed! Check IP, Username, or Password."
        exit 1
    fi
fi

echo "🔓 Allowing X11 display access..."
xhost +

echo "🚀 Starting Docker Container..."

docker run -it --rm \
  --runtime nvidia \
  --network host \
  --ipc=host \
  --privileged \
  --name jetson_ai \
  -v "$LOCAL_MOUNT_POINT":/app \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=:0 \
  -e QT_X11_NO_MITSHM=1 \
  $DOCKER_IMAGE \
  /bin/bash


