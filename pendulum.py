import pybullet as p
import numpy as np
import matplotlib.pyplot as plt

dt = 1/240                                  #Шаг симуляции pybullet
th0 = np.deg2rad(15)                        #Начальный угол отклонения маятника (его положение)
thd = np.deg2rad(30)                        #Конечный угол отклонения маятника (его положение)
T = 2                                       #Время, за которе маятник должен достигнуть конечного положения
jIdx = 1                                    #Индекс джоинта, которым управляем
maxTime = 10                                #Время симуляции
logTime = np.arange(0.0, maxTime, dt)       #Массив временных интервалов
sz = len(logTime)                           #Размер массива
logPos = np.zeros(sz)                       #Массив положений маятника в момент t
logPos[0] = np.rad2deg(th0)
logVel = np.zeros(sz)                       #Массив скоростей маятника в момент t
logAcc = np.zeros(sz)                       #Массив ускорений маятника в момент t
logCtrl = np.zeros(sz)                      #Массив упраляющих сил, действующих на маятник в момент t
idx = 0
u = 0

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
    # p.setJointMotorControl2(bodyIndex=boxId, jointIndex=jIdx, controlMode=p.TORQUE_CONTROL, force=f)
    jointState = p.getJointState(boxId, jIdx)

    s = a3*t**3 + a4*t**4 + a5*t**5
    u = th0 + s*(thd-th0)
    
    if t > T:
        u=thd
    dth1 = jointState[1]
    acc = (dth1 - dth1_prev)/dt
    dth1_prev = dth1
    logAcc[idx] = np.rad2deg(acc)
    logCtrl[idx] = u
    p.setJointMotorControl2(
        bodyIndex=boxId, 
        jointIndex=jIdx, 
        controlMode=p.POSITION_CONTROL, 
        targetPosition=u
    )

    p.stepSimulation()

    jointState = p.getJointState(boxId, jIdx)
    th1 = jointState[0]
    dth1 = jointState[1]
    logVel[idx] = np.rad2deg(dth1)
    idx += 1
    logPos[idx] = np.rad2deg(th1)

logVel[idx] = p.getJointState(boxId, jIdx)[1]
logAcc[idx] = np.rad2deg(acc)

plt.subplot(4,1,1)
plt.grid(True)
plt.plot(logTime, logPos, label = "simPos")
plt.plot([logTime[0],logTime[-1]],[np.rad2deg(thd),np.rad2deg(thd)],'r', label='refPos')
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