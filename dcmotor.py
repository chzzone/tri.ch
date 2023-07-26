from gpiozero import DistanceSensor, Motor, Button
A1A_PIN = 23
A1B_PIN = 24
########movemotor###########
def move_motor(speed):
    motor = Motor(forward=A1A_PIN, backward=A1B_PIN)
    # 모터 제어 함수
    if speed > 0:
        motor.forward(speed)
    elif speed < 0:
        motor.backward(abs(speed))
    else:
        motor.stop()