import numpy as np
import matplotlib.pyplot as plt

class Engine:
    def __init__(self, K, a1, a2, dt):
        self.update_coeficients(K, a1, a2, dt)

        self.w1 = 1.0
        self.w2 = 1.0

    def update_coeficients(self, K, a1, a2, dt):
        self.a1 = a1
        self.a2 = a2

        self.dt = dt

        self.b0 = K

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


dt = 0.01
total_time = 5
steps = int(total_time / dt)

tpoints = np.arange(steps) * dt

print("input engine coeficients, K, a1, a2")
K = float(input())
a1 = float(input())
a2 = float(input())

engine = Engine(K, a1, a2, dt)

print("input Kp, Ki, Kd")
Kp = float(input())
Ki = float(input())
Kd = float(input())

print("input SP")
SP = float(input())

pid = CanonicalPID(Kp, Ki, 0, -10, 10, dt)

spoints = np.full(steps, SP)
ppoints = np.zeros(steps)
opoints = np.zeros(steps)

PV = 0.0

for i in range(steps):
    error = spoints[i] - PV
    control = pid.compute(error)


    PV = engine.compute(control)

    if(i == 2):
        pid.update_coeficients(Kp, Ki, Kd, -10, 10, dt)
        PV = 0.0

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
1.0
-1.9
0.902
0.5
0.2
0.01
10


"""