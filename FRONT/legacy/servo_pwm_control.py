import time

import RPi.GPIO as GPIO


# 라즈베리파이 GPIO 번호 체계는 BCM 모드를 사용합니다.
# GPIO13에 서보모터 signal 선을 연결했으므로 BCM 13번을 사용합니다.
SERVO_PIN = 13

# MG995/MG995R 서보모터는 일반적으로 50Hz PWM 신호로 제어합니다.
PWM_FREQUENCY_HZ = 50

# 서보모터 각도를 만들 때 사용하는 펄스 폭 범위입니다.
# 0도 또는 180도에서 정확히 멈추지 않거나 떨림이 있으면 이 값을 조금 조정하세요.
MIN_PULSE_MS = 0.5
MAX_PULSE_MS = 2.5

# 시작 각도입니다.
START_ANGLE = 0

# 이동할 목표 각도입니다.
TARGET_ANGLE = 180

# 180도에 도착한 뒤 유지할 시간입니다.
HOLD_SECONDS = 1

# 각도를 변경한 뒤 서보모터가 움직일 시간을 기다리는 값입니다.
SETTLE_SECONDS = 0.2


def angle_to_duty_cycle(angle):
    """Convert a servo angle from 0-180 degrees to a PWM duty cycle."""
    angle = max(0, min(180, angle))
    pulse_ms = MIN_PULSE_MS + (angle / 180.0) * (MAX_PULSE_MS - MIN_PULSE_MS)
    period_ms = 1000.0 / PWM_FREQUENCY_HZ
    return (pulse_ms / period_ms) * 100.0


def set_servo_angle(pwm, angle):
    duty_cycle = angle_to_duty_cycle(angle)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(SETTLE_SECONDS)
    pwm.ChangeDutyCycle(0)


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)

    pwm = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY_HZ)

    try:
        pwm.start(0)

        set_servo_angle(pwm, START_ANGLE)
        set_servo_angle(pwm, TARGET_ANGLE)
        time.sleep(HOLD_SECONDS)
        set_servo_angle(pwm, START_ANGLE)

    finally:
        pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
