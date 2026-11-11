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

    def compute(self, error, correction):
        adj_error = error - correction

        w0 = adj_error - self.a1 * self.w1 - self.a2 * self.w2
        v = self.b0 * w0 + self.b1 * self.w1 + self.b2 * self.w2

        output = max(self.min_output, min(self.max_output, v))

        sat_diff = v - output

        self.w2 = self.w1
        self.w1 = w0

        return output, sat_diff


dt = 0.001
total_time = 100
steps = int(total_time / dt)

tpoints = np.arange(steps) * dt

"""
print("input engine coeficients, K, T1, T2")
K = float(input())
T1 = float(input())
T2 = float(input())


print("input Kp, Ki, Kd, control limit and observer constant")
Kp = float(input())
Ki = float(input())
Kd = float(input())
lim = float(input())
obs_const = float(input())

print("input SP")
SP = float(input())
"""

params = []
f = open("input.txt", "r")
for line in f:
    cleaned = line.strip()
    if cleaned:
        params.append(float(cleaned))

K = params[0]
T1 = params[1]
T2 = params[2]
Kp = params[3]
Ki = params[4]
Kd = params[5]
lim = params[6]
obs_const = params[7]
SP = params[8]

engine = Engine(K, T1, T2, dt)

pid = CanonicalPID(Kp, 0, 0, -lim, lim, dt)

spoints = np.full(steps, SP)
ppoints = np.zeros(steps)
opoints = np.zeros(steps)

PV = 0.0
sat_diff = 0.0

for i in range(steps):
    error = SP - PV
    control, sat_diff = pid.compute(error, sat_diff * Kp / Ki * obs_const)


    PV += engine.compute(control)

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
2.0
0.2
0.1
2.0
0.5
0.5
10.0
0.0
10.0


"""