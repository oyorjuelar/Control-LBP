import gym
import gym_cartpole_swingup
import math
import numpy
import Jacobian

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
    obs = mov(env,-0.8)
while (not test(ang(obs),-1.3,0.1)):
    obs = mov(env,0)

l = Jacobian.LQRT([-1.6,1.6],[-8,8],0.1,'BL.txt')
#l.setTab2File()
while True:
    K = l.getK(ang(obs),vel(obs),'BL.txt')
    #K = l.getK(0,0,'BL.txt')   
    x = [obs[0],obs[1],ang(obs),vel(obs)]
    u = numpy.dot(K,x)
    u = usat(-u,10)
    obs = mov(env,u)
    env.render()




env.close()