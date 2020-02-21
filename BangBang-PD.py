import gym
import gym_cartpole_swingup
import math

env = gym.make('CartPoleSwingUp-v0')
obs= env.reset()

ref = 0
kp = 4
kd = 0.9
e_ant = 0
h = 0.02
kd2 = kd/h

def aprox(u):
    if u>0:
        u = -1
    elif u<0:
        u = 1
    return u

def ang(obs): return math.atan2(obs[3],obs[2])
def vel(obs): return obs[4]
def test(a,b,e): 
    if math.fabs(a-b)>e:
        return False
    else:
        return True

def mov(env,ct):
    obs, rew, done, info = env.step(ct)
    print(ang(obs),vel(obs))
    env.render()
    return obs

def usat(u,mx):
    uf = (u/mx)
    if uf>1:
        uf = 1
    if uf<-1:
        uf = -1
    return uf 

for _ in range(50):
    obs = mov(env,0.5)
while (not test(math.fabs(vel(obs)),0.1,0.1)) or ang(obs)>3:
    obs = mov(env,0.9)
while (not test(ang(obs),-2,0.1)):
    obs = mov(env,-0.9)
while (not test(ang(obs),-0.24,0.1)):
    obs = mov(env,0)

while True:
    print('PD')
    e = ref - ang(obs) 
    P = kp*e
    D = kd2*(e-e_ant)
    u = P+D
    e_ant = e
    u = usat(u,0.2)
    obs,reward,done,info = env.step(u) # 0 - Izq, 1 - Der
    print(u)
    env.render()
env.close()



