import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

n=1e5
radius = 5
u_min = -radius
u_max = radius

x = np.random.uniform(u_min, u_max, int(n))
y = np.random.uniform(u_min, u_max, int(n))

# Monte Carlo integration
y = integrand(x)
y_bar = (b-a)/ x.shape[0] * y.sum()

#Actual Integration 
I = quad(integrand, a, b,)
print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
plt.scatter(x, y)

inside_x,  inside_y  = x[np.sqrt(x*x+y*y)<=radius],y[np.sqrt(x*x+y*y)<=radius]
outside_x, outside_y = x[np.sqrt(x*x+y*y)>radius],y[np.sqrt(x*x+y*y)>radius]

fig, ax = plt.subplots(1)
ax.scatter(inside_x, inside_y, c='b', alpha=0.8, edgecolor=None)
ax.scatter(outside_x, outside_y, c='r', alpha=0.8, edgecolor=None)
area = (inside_x.shape[0]/n)*((u_max-u_min)*(u_max-u_min))
value_of_pi = area/radius**2
print(value_of_pi)
plt.show()