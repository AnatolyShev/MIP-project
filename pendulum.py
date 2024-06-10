import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

dt = 1/240                                  #Шаг симуляции pybullet
th0 = 0.1                                   #Начальный угол отклонения маятника (его положение)
thd = 1.0                                   #Конечный угол отклонения маятника (его положение)
T = 2                                       #Время, за которе маятник должен достигнуть конечного положения
jIdx = 1                                    #Индекс джоинта, которым управляем
maxTime = 10                                #Время симуляции
logTime = np.arange(0.0, maxTime, dt)       #Массив временных интервалов
sz = len(logTime)                           #Размер массива
logPos = np.zeros(sz)                       #Массив положений маятника в момент t
logPos[0] = th0
logVel = np.zeros(sz)                       #Массив скоростей маятника в момент t
logAcc = np.zeros(sz)                       #Массив ускорений маятника в момент t
logCtrl = np.zeros(sz)                      #Массив упраляющих сил, действующих на маятник в момент t
idx = 0
thau = 0                                    #Начальная управляющая сила
kp = 20                                     #Коэффициент пропорциональности
kd = 10                                     #Коэффициент производной составляющей
l = 0.5                                     #Длина маятника
m = 1                                       #Масса груза маятника
g = 10                                      #Ускорение свободного падения
k = 0.5                                     #Коэфициент демпфирования

physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version
p.setGravity(0,0,-10)
boxId = p.loadURDF("./pendulum.urdf", useFixedBase=True)

# turn off internal damping
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetPosition=th0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
dth1_prev = 0

#Коэффициенты, вычисленные для полинома 5-го порядка,
# для более плавного изменения ускорения маятника (a0=a1=a2=0)
a3 = 10/T**3
a4 = -15/T**4
a5 = 6/T**5

for t in logTime[1:]:
    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]

    #Считаем управляющую силу, исходя из отклонений
    # от желаемых положения, скорости и ускорения маятника в момент t
    #При t >= T thau остаётся неизменной, фиксируя маятник в заданном положении за счёт компенсации силы тяжести
    if t < T:
        thau_d = a3*t**3 + a4*t**4 + a5*t**5
        thaud_d = 3.0*a3*t**2 + 4.0*a4*t**3 + 5.0*a5*t**4
        thaudd_d = 6.0*a3*t + 12.0*a4*t**2 + 20.0*a5*t**3

        err = th1 - thau_d 
        err_d = dth1 - thaud_d
        u = -kp * err - kd*err_d + thaudd_d
        thau = m*g*l*np.sin(th1) + k*dth1 + m*l*l*(u)

    acc = (dth1 - dth1_prev)/dt
    dth1_prev = dth1
    logAcc[idx] = acc
    logCtrl[idx] = thau

    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx, 
        controlMode=p.TORQUE_CONTROL,
        force=thau
    )

    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = dth1
    idx += 1
    logPos[idx] = th1

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logAcc[idx] = acc
logCtrl[idx] = thau

plt.subplot(4,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0],logTime[-1]],[thd,thd],'r', label='refPos')
plt.legend()

plt.subplot(4,1,2)
plt.grid(True)
plt.plot(logTime, logVel, label = "simVel")
plt.legend()

plt.subplot(4,1,3)
plt.grid(True)
plt.plot(logTime, logAcc, label = "simAcc")
plt.legend()

plt.subplot(4,1,4)
plt.grid(True)
plt.plot(logTime, logCtrl, label = "simCtrl")
plt.legend()

plt.show()
p.disconnect()