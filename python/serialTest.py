import serial
import struct #binary data -> 정수로 parsing  or 정수 -> binary data로 parsing
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

ser = serial.Serial("COM7", 115200)

fig, ax = plt.subplots()
data = deque(maxlen = 100)

def updateGraph(fram, str="hello world"):
#while True:
    #if keyboard.is_pressed("q"):
        #break
    if ser.in_waiting > 0: #시리얼 데이터 확인
        bytesData = ser.read() # 시리얼 데이터 읽기
        if bytesData == b'\x02':
            count = 1
            value = 0
            while bytesData != b'\x03':
                if ser.in_waiting >0:
                    mask = b'\x7f'
                    bytesData = ser.read()
                    count += 1
                    if count == 4:
                        bytesData = bytes([bytesData[0] & mask[0]])
                        value = value + int.from_bytes(bytesData)
                    elif count == 5:
                        bytesData = bytes([bytesData[0] & mask[0]])
                        value = value + (int.from_bytes(bytesData) << 7)
                    elif count == 6:
                        bytesData = bytes([bytesData[0] & mask[0]])
                        value = value + (int.from_bytes(bytesData) << 14)
                    elif count == 7:
                        bytesData = bytes([bytesData[0] & mask[0]])
                        value = value + (int.from_bytes(bytesData) << 21)

                        data.append(value)                        
                        ax.clear()
                        ax.plot(data)

            """
            if ser.in_waiting > 7:
                bytesData = ser.read(7)
                trimData = bytesData[2:6]

                # 데이터 파싱 (바이트 어레이)
                #data = struct.unpack('<I', trimData)[0] #unpack: binary -> integer
                # 'I' = unsiged integer

                # 데이터 파싱 (바이트)
                # 현재 trinData의 형식 :  bytes
                value = int.from_bytes(trimData, "little")
                data.append(value)

                ax.clear()
                ax.plot(data)"""
                
                # print(data)

ani = animation.FuncAnimation(fig, updateGraph, interval = 10) #애니메이션 등록
plt.show() # display
                
ser.close()

