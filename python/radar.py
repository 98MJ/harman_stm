import tkinter as tk
import math
import serial
import time

WIDTH = 640
HEIGHT = 480
angle = 0
direction = 0
objects = [[0,0],[10,0],[20,0],[30,0],[40,0],[50,0],[60,0],[70,0],[80,0],[90,0],
           [100,0],[110,0],[120,0],[130,0],[140,0],[150,0],[160,0],[170,0],[180,0]]
ser = serial.Serial("COM7", 115200)

#def btcmd():
#    print("helow world")
def drawObject(angle, distance):
    radius = WIDTH / 2
    x = radius + math.cos(angle * math.pi / 180) * distance
    y = radius - math.sin(angle * math.pi / 180) * distance
    canvas.create_oval(x-5, y-5, x+5, y+5, fill='green')


def updataScan():
    global angle #전역변수 선언된 변수의 값을 바꾸기 위해 global 사용
    global direction
    global objects
    global sendingAngle
    receiveDistance = 0
    if angle % 10 == 0: #10도 단위로 값 요청
        sendingAngle = angle
        mask = b'\x7f'
        ser.write(bytes(bytearray([0x02,0x52])))
        angleH = (angle >> 7) + 128 #0x80
        angleL = (angle & mask[0]) + 128
        crc = (0x02 + 0x52 + angleH + angleL) % 256
        ser.write(bytes(bytearray([angleH, angleL, crc, 0x03])))
    # 거리 수신
    if ser.in_waiting > 0:
        data = ser.read()
        if data == b'\x02':
            timeout = time.time() + 0.002 #두번째 바이트 수신 대기
            lostData = False
            while ser.in_waiting < 5:
                if time.time() > timeout:
                    lostData = True
                    break
            if lostData == False: # 데이터를 수신하면
                data = ser.read(5) #0x02는 아마 위에서 수신, 0번이 cmd, 1번부터 데이터
                if data[0] == 65:
                    crc = (2 + data[0] + data[1] + data[2]) % 256
                    if crc == data[3]:
                        if data[4] == 3:
                            # 데이터 파싱
                            mask = b'\x7f'
                            data_one = bytes([data[1] & mask[0]])
                            receiveDistance = int.from_bytes(data_one) << 7
                            data_one = bytes(data[2] & mask[0])
                            receiveDistance += int.from_bytes(data_one)
                            # 물체 업데이트
                            for obj in objects:
                                if obj[0] == sendingAngle:
                                    obj[1] = receiveDistance   

    canvas.delete('all') # 화면 지우기
    
    radius = WIDTH / 2
    x = radius + math.cos(angle * math.pi / 180) * radius
    y = radius - math.sin(angle * math.pi / 180) * radius
    canvas.create_line(x, y, radius, radius, fill='green', width=4)

    for obj in objects:
        drawObject(obj[0], obj[1])

    if direction == 0:
        angle += 1
        if angle == 181:
            direction = 1
    else:
        angle -= 1
        if angle == -1:
            direction = 0

    canvas.after(50, updataScan) #50ms마다 updataScan 살행

# 개체 호출
root = tk.Tk()
root.title("Ultrasonic Radar")
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
#button = tk.Button(root, text="종료", command=btcmd)

#button.pack()
canvas.pack()

# 화면 표시
updataScan()
root.mainloop()