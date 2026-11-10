import numpy as np
import matplotlib.pyplot as plt

class Engine:
    def __init__(self, K, T1, T2, dt):
        self.update_coeficients(K, T1, T2, dt)

        #попередні стани
        self.w1 = 0.0
        self.w2 = 0.0

    def update_coeficients(self, K, T1, T2, dt):
        alpha1 = dt / T1
        alpha2 = dt / T2

        self.a1 = -(2.0 - alpha1 - alpha2) / (1.0 + alpha1 + alpha2)
        self.a2 = (1.0) / (1.0 + alpha1 + alpha2)

        self.b0 = K * (alpha1 * alpha2) / (1.0 + alpha1 + alpha2)

    def compute(self, control):
        power = self.b0 * control - self.a1 * self.w1 - self.a2 * self.w2

        self.w2 = self.w1
        self.w1 = power

        return power

class CanonicalPID:
    def __init__(self, Kp, Ki, Kd, min_output, max_output, dt):
        self.a1 = -1.0
        self.a2 = 0.0

        self.update_coeficients(Kp, Ki, Kd, min_output, max_output, dt)

        self.w1 = 0.0
        self.w2 = 0.0

    def update_coeficients(self, Kp, Ki, Kd, min_output, max_output, dt):
        self.dt = dt
        self.max_output, self.min_output = max_output, min_output

        self.b0 = Kp + Ki * dt + Kd / dt
        self.b1 = -Kp - 2.0 * Kd / dt
        self.b2 = Kd / dt

    def compute(self, error):
        w0 = error - self.a1 * self.w1 - self.a2 * self.w2
        output = self.b0 * w0 + self.b1 * self.w1 + self.b2 * self.w2

        self.w2 = self.w1
        self.w1 = w0

        return max(self.min_output, min(self.max_output, output))


lim = 1000
dt = 0.001
total_time = 50
steps = int(total_time / dt)

tpoints = np.arange(steps) * dt

print("input engine coeficients, K, T1, T2")
K = float(input())
T1 = float(input())
T2 = float(input())

engine = Engine(K, T1, T2, dt)

print("input Kp, Ki, Kd")
Kp = float(input())
Ki = float(input())
Kd = float(input())

print("input SP")
SP = float(input())

pid = CanonicalPID(Kp, 0, 0, -lim, lim, dt)

spoints = np.full(steps, SP)
ppoints = np.zeros(steps)
opoints = np.zeros(steps)

PV = 0.0

for i in range(steps):
    error = SP - PV
    control = pid.compute(error)


    PV = engine.compute(control)

    if(i == 1):
        pid.update_coeficients(Kp, Ki, 0, -lim, lim, dt)

    if(i == 2):
        pid.update_coeficients(Kp, Ki, Kd, -lim, lim, dt)

    ppoints[i] = control
    opoints[i] = PV

#показ плоту написаний ШІ
plt.figure(figsize=(10, 5))
plt.plot(tpoints, spoints, 'r--', label='Уставка (SP)')
plt.plot(tpoints, opoints, 'b-', label='Вихід об\'єкта (PV / opoints)')
plt.plot(tpoints, ppoints, 'g:', label='Сигнал ПІД (Control / ppoints)')
plt.title('Симуляція ПІД-регулятора (Канонічна форма)')
plt.xlabel('Час (с)')
plt.ylabel('Значення')
plt.grid(True)
plt.legend()
plt.show()
"""
10.0
0.04
0.04
6.0
3.0
0.2
10


"""