import gym
import time

# Declaración de variables
env = gym.make('CartPole-v1')
observation = env.reset()
ref = 0
kp = 1.8
kd = 0.1
e_ant = 0
h = 0.01
kd2 = kd/h
done = False

#Función para conversión de u a 0,1
def aprox(u):
    if u>0:
        u = 0
    elif u<0:
        u = 1
    return u

#Loop, aplicación de controaldor PD
while True:
    e = ref - observation[2] 
    P = kp*e
    D = kd2*(e-e_ant)
    u = P+D
    e_ant = e
    observation,reward,done,info = env.step(aprox(u)) # 0 - Izq, 1 - Der
    env.render()
env.close()


