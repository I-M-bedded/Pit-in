from dataclasses import dataclass, field
from typing import List, Dict, Any
from fastech import servolist as sl


DEFAULT_IP_ADDRESSES = sl.udp_addresses

class InputManager:
    """
    조이스틱/키보드 입력을 통합 관리하는 클래스.
    Config 객체를 참조하여 임계값(Threshold) 등을 동적으로 가져옵니다.
    """
    def __init__(self, config):
        self.cfg = config  # RobotConfig 인스턴스를 저장
        self.raw_axes = [0.0] * 6
        self.raw_buttons = [0] * 11
        self.raw_hat = [0, 0]

    def update(self, axes: List[float], buttons: List[int], hat: List[int]):
        """센서/조이스틱 등에서 받아온 Raw 데이터를 업데이트"""
        self.raw_axes = axes
        self.raw_buttons = buttons
        self.raw_hat = hat

    def get_state(self) -> Dict[str, Any]:
        """논리적인 버튼 이름과 값을 딕셔너리로 반환"""
        # Config에서 설정된 임계값 실시간 참조
        threshold = self.cfg.trigger_threshold
        
        btns = self.raw_buttons
        axes = self.raw_axes
        hat = self.raw_hat

        return {
            # --- Buttons (Digital) ---
            'A': btns[0], 'B': btns[1], 'X': btns[2], 'Y': btns[3],
            'L': btns[4], 'R': btns[5],
            'SEL': btns[6], 'START': btns[7], 'LOGO': btns[8],
            'L_JOY_BTN': btns[9], 'R_JOY_BTN': btns[10],

            # --- D-Pad (Hat) ---
            'HAT_X': hat[0], 'HAT_Y': hat[1],

            # --- Axes (Analog) ---
            'AXIS_LX': axes[0], 'AXIS_LY': axes[1],
            'AXIS_RX': axes[2], 'AXIS_RY': axes[3],
            'LT_VAL': axes[4] if len(axes) > 4 else 0.0,
            'RT_VAL': axes[5] if len(axes) > 5 else 0.0,

            # --- Triggers (Boolean Logic) ---
            # axes 값이 threshold보다 크면 True로 인식
            'LT_PRESSED': (axes[4] > threshold) if len(axes) > 4 else False,
            'RT_PRESSED': (axes[5] > threshold) if len(axes) > 5 else False
        }

# ---------------------------------------------------------
# 2. Robot Configuration (Dataclass)
# ---------------------------------------------------------
@dataclass
class RobotConfig:
    """
    로봇의 모든 설정값과 하위 모듈(InputManager)을 관리하는 설정 클래스.
    """
    # --- System ---
    is_real_robot: bool = True
    version: int = 4
    
    # --- Network (Mutable Default는 field 사용 필수) ---
    ip_addresses: List[str] = field(default_factory=lambda: DEFAULT_IP_ADDRESSES)

    # --- Servo Heights (Unit: pulse) ---
    pin_h_bt_carry: int = 180000
    pin_h_pit_1st_bt_detach: int = 120000
    pin_h_st_attach: int = 46000
    max_pin_height_small: int = 270000
    max_pin_height_big: int = 733000

    # --- Control Parameters ---
    # 리스트 같은 가변 객체는 field(default_factory=...) 사용
    t_vel_default: List[int] = field(default_factory=lambda: [10000, 10000, 2000, 600, 600, 50000, 50000])
    pause_ratio: List[int] = field(default_factory=lambda: [100] * 14)
    
    factor_distance: int = 1
    factor_rotation: int = 100
    factor_lift: int = 10000
    factor_lift_rotate: int = 500
    factor_pin_adjust: int = 10000

    # --- Vision Parameters ---
    vision_offset_l: List[float] = field(default_factory=lambda: [-5.7, 3.2])
    cam_l_rotation: List[List[float]] = field(default_factory=lambda: [[0, 1], [-1, 0]])

    # --- Timeouts & Thresholds ---
    trigger_threshold: float = 0.8
    homing_timeout_step_1: float = 2.0
    loop_sleep_servo: float = 0.16
    loop_sleep_main: float = 0.04

    # --- Internal Modules (초기화 시 값 입력 안 받음) ---
    input: InputManager = field(init=False)

    def __post_init__(self):
        """
        __init__이 실행된 직후 자동으로 호출되는 메서드.
        여기서 InputManager를 생성하고 자기 자신(self)을 주입합니다.
        """
        self.input = InputManager(self)