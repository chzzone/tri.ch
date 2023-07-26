from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import InfraredSensor
from time import sleep

ir_receiver_pin = 27
ir_receiver = InfraredSensor(ir_receiver_pin, pin_factory=factory)

def print_button_press(button_code):
    # 버튼 코드를 출력하는 함수
    print(f"Button Pressed: {hex(button_code)}")

try:
    while True:
        if ir_receiver.is_active:
            # 적외선 신호를 감지하면 버튼 코드를 출력합니다.
            print_button_press(ir_receiver.value)

        sleep(0.1)
except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")