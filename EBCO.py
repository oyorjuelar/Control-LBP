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

def PD(ref,obs,e_ant,kp,kd):
    h = 0.02
    kd2 = kd/h
    e = ref - obs 
    P = kp*e
    D = kd2*(e-e_ant)
    u = P+D
    e_ant = e
    a[0] = -u
    a[1] = e_ant
    return a

def eventFunction(ob1,ob2): return math.fabs((sign(ob1[3]*math.cos(ob1[2]))) - (sign(ob2[3]*math.cos(ob2[2]))))

def feedBackFunction(ob1):
    ap = 8
    e = 0
    m = 0.1
    l = 0.5
    I = ((1/12)*m*(l**2))+(m*((l/2)**2))
    g = 9.8
    E = ((1/2)*I*(ob1[3]**2))+(m*g*l*(math.cos(ob1[2])-1))
    return -ap*(E-e)*sign(ob1[3]*math.cos(ob1[2]))

def eventBasedController(ob1,ob2):    
    ev = eventFunction(ob1,ob2)
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
a = [0,0]

for _ in range(1000):
    if math.fabs(obi[2]) < 0.11:
        print('PD')
        a = PD(0,obi[2],a[1],2,0.04)
        u = aprox(a[0])
        vel1 = obi[2]    
        obi,reward,done,info = env.step(u) # 0 - Izq, 1 - Der
        vel2 = obi[2]
        ace = (vel2-vel1)*50
        env.render()
    else:

        print('EBC')
        obf = eventBasedController(obi,obf)
        v = obf[4] 
        
        while True:
            vel1 = obi[2]
            if v>ace:
                n = 0
                obi,reward,done,info = env.step(n) # 0 - Izq, 1 - Der
            else:
                n = 1
                obi,reward,done,info = env.step(n) # 0 - Izq, 1 - Der
            vel2 = obi[2]
            ace = (vel2-vel1)*50
            env.render()
            if(math.fabs(v-ace) > 1):
                break

env.close()