import gym
import time
import math
import numpy

# Declaración de variables
"""
observation[0] = Posición horizontal
observation[1] = Velocidad horizontal
observation[2] = Ángulo poste
observation[3] = Velocidad angular poste
"""

def sign(x): return math.copysign(1, x)



#Función para conversión de u a 0,1
def aprox(u):
    if u>0:
        u = 1
    elif u<=0:
        u = 0
    return u

def PD(ref,env,obs):
    kp = 1
    kd = 0.5
    e_ant = 0
    h = 0.01
    kd2 = kd/h
    e = ref - obs[1] 
    print(e)
    while math.fabs(e)>0.01:
        e = ref - obs[1] 
        P = kp*e
        D = kd2*(e-e_ant)
        u = P+D
        e_ant = e
        obs,reward,done,info = env.step(aprox(u)) # 0 - Izq, 1 - Der
        env.render()
        time.sleep(h)
    return obs

def eventFunction(ob1,ob2): return math.fabs((sign(ob1[3]*math.cos(ob1[2]))) - (sign(ob2[3]*math.cos(ob2[2]))))

def feedBackFunction(ob1):
    ap = 0.01
    I = 0.01
    g = 9.8
    m = 0.1
    l = 0.5
    e = 0
    E = (0.5*I*(ob1[3]**2))+(m*g*l*(math.cos(ob1[2])-1))
    return -ap*(E-e)*sign(ob1[3]*math.cos(ob1[2]))
    
def eventBasedController(ob1,ob2):    
    ev = eventFunction(ob1,ob2)
    print(ev)
    if ev == 0:
        gm = feedBackFunction(ob1)
        ob2[0] = ob1[0]
        ob2[1] = ob1[1]
        ob2[2] = ob1[2]
        ob2[3] = ob1[3]        
        ob2[4] = gm
    else:
        pass

    return ob2

env = gym.make('CartPole-v1')
obi = env.reset()
obf = numpy.append(obi, 0)
dt = 0.01


for _ in range(5):
    env.render()
    env.step(0) # 0 - Izq, 1 - Der

for _ in range(5):
    env.render()
    obi,reward,done,info = env.step(1) # 0 - Izq, 1 - Der


for _ in range(2):
    env.render()
    env.step(0) # 0 - Izq, 1 - Der


for _ in range(500):
    env.render()
    obf = eventBasedController(obi,obf)
    #obi,reward,done,info = env.step(aprox(obf[4])) # 0 - Izq, 1 - Der
    obi = PD(obf[4],env,obi)

env.close()


#Loop, aplicación de controaldor PD
"""while not done:
    time.sleep(0.1)
"""