import pygame
import time

# 초기화
pygame.init()
pygame.joystick.init()

# Xbox 컨트롤러가 연결되어 있는지 확인
if pygame.joystick.get_count() < 1:
    print("Xbox 컨트롤러를 연결하세요.")
    pygame.quit()
    exit()

# 첫 번째 Xbox 컨트롤러 선택
controller = pygame.joystick.Joystick(0)
controller.init()
print(f"컨트롤러 이름: {controller.get_name()}")

# 입력 감지 함수
def check_input():
    pygame.event.pump()  # 이벤트 큐 새로고침

    # 조이스틱 축 (스틱) 입력 감지
    for i in range(controller.get_numaxes()):
        axis = controller.get_axis(i)
        # print(f"축 {i} 움직임: {axis:.2f}")
        if abs(axis) > 0.8:  # 축이 중립 위치를 벗어나면 감지
            print(f"축 {i} 움직임: {axis:.2f}")

    # 버튼 입력 감지
    for i in range(controller.get_numbuttons()):
        if controller.get_button(i):
            print(f"버튼 {i} 눌림")

    # 트리거 (D-Pad) 입력 감지
    hat = controller.get_hat(0)
    if hat != (0, 0):  # 중립 위치를 벗어나면 감지
        print(f"트리거 움직임: {hat}")

try:
    while True:
        check_input()
        time.sleep(0.1)  # 입력 확인 간격 조정
except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    pygame.quit()
