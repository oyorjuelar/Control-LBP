from __future__ import division, print_function
import scipy.linalg
import sympy
import random
import numpy as np
from sympy import sin, cos, Matrix
from sympy.abc import x, v, t, a, e
import math

class LQRT:

    rangoTheta = [0,0]
    rangoThetaP = [0,0]
    paso = 1
    archivo = 'Default.txt'
    tabla = dict()

    def __init__(self,rt,rtp,p,a):
        self.rangoTheta = rt
        self.rangoThetaP = rtp
        self.paso = p
        self.archivo = a
        self.tabla = self.getTabFrFile(a)


    def Lin(self):
        M = 0.5
        m = 0.5
        l = 0.6
        g = 9.82

        f1 = v
        f2 = ((-m*l*sin(t)*(a**2))+(m*g*cos(t)*sin(t)))/(M+m-(m*(cos(t)**2)))
        f3 = a
        f4 = ((-m*l*cos(t)*sin(t)*(a**2))+(m*g*sin(t))+(M*g*sin(t)))/(l*(M+m-(m*(cos(t)**2))))
        f5 = e/(M+m-(m*(cos(t)**2)))
        f6 = (e*cos(t))/(l*(M+m-(m*(cos(t)**2))))
        
        fun = Matrix([f1, f2, f3, f4])
        X = Matrix([x, v, t, a])
        Ac = fun.jacobian(X)
        fun2 = Matrix([0, f5, 0, f6])
        Bc = fun2.jacobian([e])
        
        T2 = [Ac,Bc]
        return T2

    def eval(self,T1,x1,x2):
        Ac = T1[0]
        Bc = T1[1]
        fxn = sympy.lambdify(t,Ac.subs(a, x2),'numpy')
        A = fxn(x1)
        fxn2 = sympy.lambdify(t,Bc.subs(a, x2),'numpy')
        B = fxn2(x1)
        T = [A,B]
        return T

    def lqr(self,A,B,Q,R):
        """Solve the continuous time lqr controller.
        
        dx/dt = A x + B u
        
        cost = integral x.T*Q*x + u.T*R*u
        """
        #ref Bertsekas, p.151
        
        #first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
        #compute the LQR gain
        #K = np.matrix(scipy.linalg.inv(R)*(B.T*X))
        K = np.array((B.T*X)/R)

        eigVals, eigVecs = scipy.linalg.eig(A-B*K)
    
        return K, X, eigVals

    def genTabla(self,rx1,rx2,step):
        Q = np.diag([10,1,10,1])
        R = 10
        T1 = self.Lin()
        tabla = dict()
        samples = int(math.fabs((rx1[1]-rx1[0])/step))
        samples2 = int(math.fabs((rx2[1]-rx2[0])/step))
        totalS = (samples+1)*(samples2+1)
        s = 0
        b = len(str(float(step)).split('.')[1])
        n = 10**b
        print(rx1[0]*n,rx1[1]*n,step*n)
        for i in range(samples+1):
            for j in range(samples2+1):
                s+=1
                n = round(rx1[0]+(i*step),b)
                m = round(rx2[0]+(j*step),b)
                print(str(round((s*100)/(totalS),2))+'%')
                T2 = self.eval(T1,n,m)
                k,x,egv = self.lqr(T2[0],T2[1],Q,R)
                tabla[n,m] = k
        return tabla

    def guardarDatos(self,nombreArchivo,tabla):
        archivo = open(nombreArchivo, "w")
        for i in tabla:
            archivo.write(str(i))
            archivo.write(str(tabla.get((i)))+'\n')
        archivo.close()

    def getTabFrFile(self,nombre_archivo):
        tabla = dict()
        archivo = open(nombre_archivo, "r")
        for línea in archivo:
            i= línea.rstrip().rstrip(']').split('[[')
            #print(i)
            ind = i[0].rstrip(')').split('(')[1].split(',')
            indf = [float(x) for x in ind]
            #print(i[1].split())
            K = [[float(y.rstrip('.').rstrip(',')) for y in i[1].split()]]
            tabla[indf[0],indf[1]]=K
        archivo.close()
        return tabla

    def setTab2File(self):
        tabla = dict()
        tabla = self.genTabla(self.rangoTheta,self.rangoThetaP,self.paso)
        self.guardarDatos(self.archivo,tabla)

    def aprox(self,bs,n,sp):
        x = round(((n-bs)/sp))
        b = len(str(float(sp)).split('.')[1])
        return round(bs+(x*sp),b)
        
    def getK(self,i,j,nombreArchivo):
        try:
            return self.tabla.get((i,j))[0]
        except TypeError:
            print(i,j)
            print(self.aprox(self.rangoTheta[0],i,self.paso),self.aprox(self.rangoThetaP[0],j,self.paso))
            return self.tabla.get((self.aprox(self.rangoTheta[0],i,self.paso),self.aprox(self.rangoThetaP[0],j,self.paso)))[0]