#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# GPIO pin definitions
VIBRATION_PIN = 18  # GPIO 18 (Pin 12)
SERVO_PIN = 12      # GPIO 12 (Pin 32)

def setup_gpio():
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)
    
    # Set up vibration motor pin as digital output
    GPIO.setup(VIBRATION_PIN, GPIO.OUT)
    
    # Set up servo pin
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency for servo
    
    return servo_pwm

def test_vibration_motor():
    print("Testing vibration motor...")
    try:
        # Turn motor ON
        GPIO.output(VIBRATION_PIN, GPIO.HIGH)
        print("Vibration motor should be ON")
        time.sleep(2)
        
        # Turn motor OFF
        GPIO.output(VIBRATION_PIN, GPIO.LOW)
        print("Vibration motor should be OFF")
        time.sleep(1)
        
        # Turn motor ON again
        GPIO.output(VIBRATION_PIN, GPIO.HIGH)
        print("Vibration motor should be ON again")
        time.sleep(2)
        
        # Turn motor OFF
        GPIO.output(VIBRATION_PIN, GPIO.LOW)
        print("Vibration motor stopped")
    except KeyboardInterrupt:
        GPIO.output(VIBRATION_PIN, GPIO.LOW)
        print("\nVibration test interrupted")

def test_servo_motor(pwm):
    print("\nTesting servo motor...")
    try:
        # Start PWM at neutral position (7.5% duty cycle)
        pwm.start(7.5)
        print("Servo at neutral position")
        time.sleep(1)
        
        # Move to 0 degrees (5% duty cycle)
        pwm.ChangeDutyCycle(5)
        print("Servo at 0 degrees")
        time.sleep(1)
        
        # Move to 90 degrees (7.5% duty cycle)
        pwm.ChangeDutyCycle(7.5)
        print("Servo at 90 degrees")
        time.sleep(1)
        
        # Move to 180 degrees (10% duty cycle)
        pwm.ChangeDutyCycle(10)
        print("Servo at 180 degrees")
        time.sleep(1)
        
        # Return to neutral
        pwm.ChangeDutyCycle(7.5)
        print("Servo returned to neutral")
        time.sleep(1)
        
        # Stop
        pwm.stop()
        print("Servo test complete")
    except KeyboardInterrupt:
        pwm.stop()
        print("\nServo test interrupted")

def main():
    try:
        print("GPIO Test Program")
        print("Press Ctrl+C to exit")
        
        # Set up GPIO
        servo_pwm = setup_gpio()
        
        # Test vibration motor
        test_vibration_motor()
        
        # Test servo motor
        test_servo_motor(servo_pwm)
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up GPIO
        GPIO.cleanup()
        print("\nGPIO cleanup complete")

if __name__ == "__main__":
    main() 