import io
import picamera
from flask import Flask, Response
import socket
import RPi.GPIO as GPIO
import pigpio
from time import sleep
import threading



app = Flask(__name__)
    
MotorA_in1 = 17
MotorA_in2 = 27
MotorA_enable = 5

MotorB_in1 = 22
MotorB_in2 = 23
MotorB_enable = 6

PWM_Frequency = 1000
Duty_Cycle_Increment = 5

Motor_servo_steering = 12
Motor_servo_camera = 14

Buzzer_pin = 13

def Start_Buzzer():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Buzzer_pin, GPIO.OUT)
    pwm = GPIO.PWM(Buzzer_pin, 1000)
    pwm.start(50)
    sleep(1)
    pwm.stop()
    

def generate_frames():
    with picamera.PiCamera() as camera:
        camera.resolution = (320,240) # 640, 480  
        camera.framerate = 15
        camera.vflip = True
        camera.hflip = True
        stream = io.BytesIO()

        for _ in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
            stream.seek(0)
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + stream.read() + b'\r\n'
            stream.seek(0)
            stream.truncate()
            
@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def servo_init_steering():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_servo_steering, GPIO.OUT)
    GPIO.output(Motor_servo_steering, True)

    global pwm_servo_steering
    pwm_servo_steering = pigpio.pi()
    pwm_servo_steering.set_mode(Motor_servo_steering, pigpio.OUTPUT)
    pwm_servo_steering.set_PWM_frequency(Motor_servo_steering,50)
    
    global max_servo_range_steering
    max_servo_range_steering = 2300
    global min_servo_range_steering
    min_servo_range_steering = 1200
    
def servo_init_camera():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Motor_servo_camera, GPIO.OUT)
    GPIO.output(Motor_servo_camera, True)

    global pwm_servo_camera
    pwm_servo_camera = pigpio.pi()
    pwm_servo_camera.set_mode(Motor_servo_camera,pigpio.OUTPUT)
    pwm_servo_camera.set_PWM_frequency(Motor_servo_camera,50)
    
    global max_servo_range_camera
    max_servo_range_camera = 2500
    global min_servo_range_camera
    min_servo_range_camera = 800
    
    global camera_angle
    camera_angle=1700
    setAngle_camera(camera_angle)
    
    
def setAngle_steering(angle):
    duty = angle / 18 + 2
    pwm_servo_steering.set_servo_pulsewidth(Motor_servo_steering,angle)

def setAngle_camera(angle):
    duty = angle / 18 + 2
    pwm_servo_camera.set_servo_pulsewidth(Motor_servo_camera,angle)


def server_socket_init():
    host = '172.30.170.224'
    port = 6789

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_socket.bind((host, port))

    server_socket.listen()

    print("Server listening on", host, "port", port)

    client_socket, client_address = server_socket.accept()
    print("Connection from:", client_address)

    return server_socket, client_socket


def server_socket_receive(client_socket):

    data = client_socket.recv(1024).decode()
    
    return data


def server_socket_close_connection(client_socket):

    client_socket.close()


def parse_data(data):
    parsed_data = data.strip().split(',')
    try:
        useful_data_servo_steering= float(parsed_data[0].strip().rstrip('.'))
        useful_data_f= float(parsed_data[-2].strip().rstrip('.'))
        useful_data_r = float(parsed_data[-3].strip().rstrip('.'))
        useful_data_servo_camera = float(parsed_data[-1].strip().rstrip('.'))
        direction = 1
        if useful_data_r > -1:
            direction = 0
            return useful_data_r, direction, useful_data_servo_steering, useful_data_servo_camera
        
        return useful_data_f, direction, useful_data_servo_steering, useful_data_servo_camera
    except:
        print("Error while parsing in parse_data")


def transform_acceleration(value):
    acc_value = (value + 1) * 50
    return acc_value




def transform_servo_angle_steering(value_servo):
    old_range=(1- (-1))
    new_range=(max_servo_range_steering - min_servo_range_steering)
    angle= (((value_servo - (-1))*new_range)/old_range)+min_servo_range_steering
    
    angle=(max_servo_range_steering - (angle -min_servo_range_steering))

    return angle

def transform_servo_angle_camera(value_servo):
    global camera_angle
    
    step_size = 70

    if value_servo == -1:
        camera_angle += step_size
    elif value_servo == 1:
        camera_angle -= step_size

    camera_angle = max(min(camera_angle,max_servo_range_camera), min_servo_range_camera)



def motor_init():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MotorA_in1, GPIO.OUT)
    GPIO.setup(MotorA_in2, GPIO.OUT)
    GPIO.setup(MotorA_enable, GPIO.OUT)
    GPIO.setup(MotorB_in1, GPIO.OUT)
    GPIO.setup(MotorB_in2, GPIO.OUT)
    GPIO.setup(MotorB_enable, GPIO.OUT)

    global pwmA
    global pwmB
    pwmA = GPIO.PWM(MotorA_enable, PWM_Frequency)
    pwmB = GPIO.PWM(MotorB_enable, PWM_Frequency)
    pwmA.start(0)
    pwmB.start(0)


def motor_direction(direction, acceleration):

    if direction == 1:
        # Going forwards
        GPIO.output(MotorA_in1, GPIO.HIGH)
        GPIO.output(MotorA_in2, GPIO.LOW)
        GPIO.output(MotorA_enable, GPIO.HIGH)
        GPIO.output(MotorB_in1, GPIO.HIGH)
        GPIO.output(MotorB_in2, GPIO.LOW)
        GPIO.output(MotorB_enable, GPIO.HIGH)

        pwmA.ChangeDutyCycle(acceleration)
        pwmB.ChangeDutyCycle(acceleration)

    if direction == 0:
        # Going backwards
        GPIO.output(MotorA_in1, GPIO.LOW)
        GPIO.output(MotorA_in2, GPIO.HIGH)
        GPIO.output(MotorA_enable, GPIO.HIGH)
        GPIO.output(MotorB_in1, GPIO.LOW)
        GPIO.output(MotorB_in2, GPIO.HIGH)
        GPIO.output(MotorB_enable, GPIO.HIGH)
        try:
            pwmA.ChangeDutyCycle(acceleration)
            pwmB.ChangeDutyCycle(acceleration)
        except:
            print("Error pwm")


def all_motor_stop():
    pwm_servo_steering.stop()
    pwm_servo_camera.stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()


def main():
    
    sleep(60)
    Start_Buzzer()

    sever_socket, client_socket = server_socket_init()

    sleep(1)
    motor_init()
    servo_init_steering()
    servo_init_camera()
    


    var_loop_on_off = True
    var = False
    while var_loop_on_off:
        
        data = server_socket_receive(client_socket)
        if not data:
            break
        try:
            
            useful_data, direction, servo_angle_steering, servo_angle_camera = parse_data(data)
            acceleration = transform_acceleration(useful_data)
            motor_direction(direction, acceleration)
            transformed_angle_steering = transform_servo_angle_steering(servo_angle_steering)
            setAngle_steering(transformed_angle_steering)
            

            transform_servo_angle_camera(servo_angle_camera)
            setAngle_camera(camera_angle)
            print(camera_angle)
            sleep(0.1)
            
        except:
            print("Error while parsing(main)")
            


        if var == True:
            var_loop_on_off = False
            motor_stop()
            
    all_motor_stop()
    server_socket_close_connection(client_socket)


if __name__ == '__main__':

    t1 = threading.Thread(target=app.run, kwargs={'host':'0.0.0.0', 'port':6700,'threaded':True})
    t1.start()
    
    t2=threading.Thread(target=main)
    t2.start()
