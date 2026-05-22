import sys
import time
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from src_robot.fastech import protocol as fs
from src_robot.fastech import servolist as sl

TEST_PULSE = 10000
TEST_SPEED = 3000

def move_and_wait(s, target, speed, wait=3.0):
    s.state_embed()       # 이동 전 상태 동기화
    time.sleep(0.1)
    res = s.servo_amove(target, speed)
    print(f"  servo_amove 반환값: {res}  (0이면 정상)")
    time.sleep(wait)
    s.state_embed()
    time.sleep(0.2)
    return s.com_apos

def main():
    print("=" * 60)
    print("   Stage X (Fwd/Back) 방향 확인 테스트 [디버깅]")
    print("=" * 60)

    addr = sl.udp_addresses[1]
    try:
        s = fs.fastech(addr)
        print(f"  연결 성공: 모터 1 ({addr})")
    except Exception as e:
        print(f"  연결 실패: {e}")
        return

    s._state_init()
    time.sleep(0.5)
    s.servo_on()
    print("  Servo ON - 홀딩 대기 3초...")
    time.sleep(3.0)      # 여유 있게 대기

    # 초기 위치 확인
    s.state_embed()
    time.sleep(0.2)
    start_pos = s.com_apos
    print(f"\n  초기 위치: {start_pos} Pulse")

    # ── (+) 방향 이동 ──
    target = start_pos + TEST_PULSE
    print(f"\n  → +{TEST_PULSE} Pulse 이동 (목표: {target})")
    cur = move_and_wait(s, target, TEST_SPEED)
    print(f"  실제 위치: {cur} Pulse  (이동량: {cur - start_pos})")

    d_plus = input("  ↳ 어느 방향? (앞/뒤): ").strip()

    # ── 복귀 ──
    print(f"\n  ← 복귀 중... ({start_pos})")
    cur = move_and_wait(s, start_pos, TEST_SPEED)
    print(f"  복귀 후 위치: {cur} Pulse")

    # ── (-) 방향 이동 ──
    target = start_pos - TEST_PULSE
    print(f"\n  → -{TEST_PULSE} Pulse 이동 (목표: {target})")
    cur = move_and_wait(s, target, TEST_SPEED)
    print(f"  실제 위치: {cur} Pulse  (이동량: {cur - start_pos})")

    d_minus = input("  ↳ 어느 방향? (앞/뒤): ").strip()

    # ── 최종 복귀 ──
    print(f"\n  ← 최종 복귀 중... ({start_pos})")
    move_and_wait(s, start_pos, TEST_SPEED)

    s.servo_off()
    print("\n  Servo OFF - 완료!")
    print(f"\n  [결과]")
    print(f"  Stage X  +방향 = {d_plus}")
    print(f"  Stage X  -방향 = {d_minus}")

if __name__ == '__main__':
    main()