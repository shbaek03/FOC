import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import csv
import time

# ==========================================
# [설정] 본인의 환경에 맞게 수정하세요
# ==========================================
COM_PORT = 'COM3'      # STM32 포트
BAUD_RATE = 1000000    # 1Mbps
MAX_POINTS = 300       # 그래프 표시 포인트
SAVE_FILENAME = f"foc_data_{int(time.time())}.csv" # 저장될 파일명
# ==========================================

# 데이터 버퍼 및 CSV 파일 준비
data_buffers = [deque(maxlen=MAX_POINTS) for _ in range(4)]
csv_file = open(SAVE_FILENAME, 'w', newline='')
csv_writer = csv.writer(csv_file)
# 헤더 저장
csv_writer.writerow(['1', '2', '3', '4'])

# 시리얼 연결
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
    print(f"Connected to {COM_PORT}. Saving to {SAVE_FILENAME}")
except Exception as e:
    print(f"Error: {e}")
    exit()

# 그래프 설정 (라벨/단위 제거)
fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
lines = []

for i, ax in enumerate(axes):
    line, = ax.plot([], [], label=str(i+1))
    ax.set_xlim(0, MAX_POINTS)
    ax.set_ylabel(str(i+1)) # 단위를 지우고 숫자만 표기
    ax.grid(True, alpha=0.3)
    lines.append(line)

plt.tight_layout()

def update(frame):
    try:
        while ser.in_waiting:
            raw_line = ser.readline()
            try:
                line_str = raw_line.decode('utf-8').strip()
                if not line_str: continue
                
                # 공백 기준 분리
                parts = line_str.split()
                if len(parts) == 4:
                    values = [float(p) for p in parts]
                    
                    # 1. CSV 저장
                    csv_writer.writerow(values)
                    
                    # 2. 버퍼 업데이트
                    for i in range(4):
                        data_buffers[i].append(values[i])
                        
            except (UnicodeDecodeError, ValueError):
                continue

        # 그래프 그리기
        for i in range(4):
            if data_buffers[i]:
                lines[i].set_data(range(len(data_buffers[i])), data_buffers[i])
                # Y축 범위 자동 조정
                axes[i].set_ylim(min(data_buffers[i]) - 0.1, max(data_buffers[i]) + 0.1)

    except Exception as e:
        print(f"Loop Error: {e}")

    return lines

# 애니메이션 실행
ani = FuncAnimation(fig, update, interval=10, blit=False)

try:
    plt.show()
finally:
    # 종료 시 파일 안전하게 닫기
    csv_file.close()
    ser.close()
    print(f"Data saved to {SAVE_FILENAME}")
