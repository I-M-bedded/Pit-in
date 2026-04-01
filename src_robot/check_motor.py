import sys
import time
import os

# Append project path to use absolute imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../")
from src_robot.fastech import protocol as fs
from src_robot.fastech import servolist as sl

def main():
    print("="*60)
    print("   모터 분해능 / 최소 반응 펄스(Deadband) 테스트")
    print("="*60)
    
    # 테스트할 축 번호 (0~4)
    target_motors = [0, 1, 2, 3, 4]
    motor_names = ['0: Stage Y(Left/Right)', '1: Stage X(Fwd/Back)', '2: Base Angle(rtZ)', '3: Left Wing(Lr)', '4: Right Wing(Rr)']
    # 테스트할 펄스 크기 목록
    test_steps = [10, 50, 100, 200, 500, 1000, 2000, 4000, 8000, 10000, 20000]
    
    addresses = sl.udp_addresses
    servos = []
    
    print("\n[1] 모터 드라이버 연결 중...")
    for idx in target_motors:
        addr = addresses[idx]
        try:
            s = fs.fastech(addr)
            servos.append((idx, s))
            print(f"  - 연결 성공: 모터 {idx} ({addr})")
        except Exception as e:
            print(f"  - 연결 실패: 모터 {idx} ({addr}) - {e}")
            
    if not servos:
        print("[오류] 연결된 모터가 없습니다. 프로그램을 종료합니다.")
        return

    print("\n[2] Servo ON (제어권 획득)")
    for idx, s in servos:
        s._state_init()
        time.sleep(0.1)
        s.servo_on()
        
    print("  (모터 홀딩 대기 2초...)")
    time.sleep(2.0)
    
    print("\n[3] 분해능 테스트 시작")
    for idx, s in servos:
        print(f"\n{'-'*60}")
        print(f" ▷ 테스트 축: {motor_names[idx]}")
        
        # 현재 위치 획득
        s.state_embed()
        time.sleep(0.1)
        start_pos = s.com_apos
        print(f"  현재 설정된 위치(초기값): {start_pos} Pulse")
        
        found_threshold = None
        
        for step in test_steps:
            target_pos = start_pos + step
            print(f"  ==> 명령 하달: +{step} Pulse (목표: {target_pos})")
            
            # Fastech In-position flag 업데이트
            s.state_embed()
            
            # 이동 명령 지시 (절대 좌표 이동, 속도는 5000 수준)
            res = s.servo_amove(target_pos, 5000)
            if res != 0:
                print(f"     [!] 명령 전송 실패 (Error Code: {res})")
            
            # 이동할 시간 대기
            time.sleep(0.7)
            
            # 새로운 위치 읽기
            s.state_embed()
            new_pos = s.com_apos
            moved_dist = abs(new_pos - start_pos)
            
            if moved_dist >= step * 0.8:  # 명령한 펄스의 80% 이상 도달했다면 (오차 감안)
                print(f"     [성공] 실제 도달: {new_pos} (+{moved_dist}) -> 기계가 반응합니다!")
                found_threshold = step
                break
            else:
                print(f"     [무시됨] 실제 도달: {new_pos} (+{moved_dist}) -> 반응이 없거나 너무 작습니다.")
                
        if found_threshold:
            print(f" => 모터 {idx}의 최소 응답 데드밴드는 약 [{found_threshold} Pulse] 부근입니다.")
        else:
            print(f" => 모터 {idx}이 {test_steps[-1]} Pulse 명령도 무시했습니다. 하드웨어/드라이버 오류를 점검하세요.")
            
        # 테스트 끝났으면 다시 원래 위치로 복귀
        print(f"  복귀 중... ({start_pos})")
        s.state_embed()
        s.servo_amove(start_pos, 5000)
        time.sleep(1.0)

    print("\n[4] 테스트 종료. Servo OFF")
    for idx, s in servos:
        s.servo_off()
    print("완료!")

if __name__ == '__main__':
    main()
