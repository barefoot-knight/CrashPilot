import numpy as np
import math

x = np.linspace(1,100,100)
y = np.zeros_like(x)
y = math.cos(x)


#for i in range(0, len(x)):
#    y[i] = math.cos(x[i])
print(y)

x = np.array()