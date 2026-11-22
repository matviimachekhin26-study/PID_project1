import matplotlib.pyplot as plt
import numpy as np
import time

global_time = 0
xpoints = np.array([])
ypoints = np.array([])
spoints = np.array([])
ipoints = np.array([0])

motor_time = 0


min_I = -0.5
max_I = 0.5
max_INP = 1e5
PV = 0




def engine_power_func(input : float, dt : float) -> float:
    return max(-1.0, min(1.0, input)) * dt * 10

def engine(input : float) -> float:
    global motor_time, PV, min_I, max_I, max_INP

    temp_time = motor_time
    motor_time = time.time()
    value = PV + engine_power_func(input, motor_time - temp_time)

    return value

def PID(Kp : float, Ki : float, Kd : float):
    global motor_time, PV, xpoints, ypoints, spoints, global_time, ipoints

    print("input SP")
    SP = float(input())

    
    global_time = time.time()

    xpoints = np.append(xpoints, [time.time() - global_time])
    ypoints = np.append(ypoints, [PV])
    spoints = np.append(spoints, [SP])

    prev_error = SP - PV
    motor_time = time.time()
    PV = engine(Kp * (SP - PV))
    PID_time = time.time()
    #print(PV)

    xpoints = np.append(xpoints, [time.time() - global_time])
    ypoints = np.append(ypoints, [PV])
    spoints = np.append(spoints, [SP])
    ipoints = np.append(ipoints, [Kp * (SP - PV)])

    IG = 0

    for i in range(int(2e4)):
        error = SP - PV
        temp_time = PID_time
        PID_time = time.time()
        dt = PID_time - temp_time
        IG += error * dt
        if dt > 0:
            DG = (error - prev_error) / dt
        else:
            DG = 0.0
        prev_error = error
        INP = Kp * error + max(min_I, min(max_I, Ki * IG)) + Kd * DG
        INP = max(-max_INP, min(max_INP, INP))
        PV = engine(INP)
        #if(i % 100 == 0):
        #    print(PV, INP)

        xpoints = np.append(xpoints, [time.time() - global_time])
        ypoints = np.append(ypoints, [PV])
        spoints = np.append(spoints, [SP])
        ipoints = np.append(ipoints, [INP])

        time.sleep(1e-6)


    return


print("Input Kp, Ki, Kd")
Kp = float(input())
Ki = float(input())
Kd = float(input())

PID(Kp, Ki, Kd)

plt.plot(xpoints, ypoints, xpoints, spoints, xpoints, ipoints)
plt.show()

"""
0.4
0.01
0.05
10
"""