from re import M
import numpy as np
import matplotlib.pyplot as plt

kay = 75 # W/m*K
rho = 2746 # kg/m^2
cp = 1030 #J/kg*K
alpha = kay/(rho*cp)
t = 15 # seconds

l = .06 # m
nodes = 35

boundaryConditions = 20 # degrees celcius
initialConditions = 515 # degrees Celcius

dx = l/(nodes - 1)
dt =.1

tau = alpha*dt/(2*dx**2)

x = np.linspace(0, l, nodes+2)
t = np.arange(0, t+dt, dt)
n = len(x)
m = len(t)
#print(m,n)

T = np.zeros((m,n))

# row, column
T[0,:] = initialConditions
T[:,0] = boundaryConditions
T[:,-1] = boundaryConditions
#print(T)

#print(tau)
A = np.diag([-tau]*(n-3),-1) + np.diag([2*(1+tau)]*(n-2), 0) + np.diag([-tau]*(n-3),1)

for j in range (1,m):
    b = T[j-1,1:-1].copy()
    #print(np.shape(A), np.shape(b))
    #print(b)
    for i in range(1,n-1):
        if i == 1:
            b[0] = tau*T[j-1,0] + 2*(1-tau)*T[j-1,1] + tau*T[j-1,2] + tau*T[j,0]
        elif i == n-2:
            b[-1] = tau*T[j-1,-1] + 2*(1-tau)*T[j-1,-2] + tau*T[j-1,-3] + tau*T[j,0]
        else: 
            b[i-1] = tau*T[j-1,i-1] + 2*(1-tau)*T[j-1,i] + tau*T[j-1,i+1]
            
    #print(T)
    #print(b.round(3))
    solution = np.linalg.solve(A,b)
    #print(solution)
    T[j,1:-1] = solution
    #print(T.round(3))

R = np.linspace(1,0,m)
B = np.linspace(0,1,m)
G = 0

for j in range(m):
  plt.plot(x, T[j,:], color = [R[j],G,B[j]])

plt.xlabel('distance[m]')
plt.ylabel('Temperature [%\degree$ C]')
#plt.legend([f't={value}s' for value in t])
plt.show()