import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad

def integrand(x):
    return  np.sqrt(b**2-x**2)

n=1e5
radius = 5
a = -radius
b = radius

x = np.random.uniform(a, b, int(n))
#y = np.random.uniform(u_min, u_max, int(n))

# Monte Carlo integration
y = integrand(x)
y_bar = (b-a)/ x.shape[0] * y.sum()

#Actual Integration 
I = quad(integrand, a, b,)
print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
plt.scatter(x, y)
plt.title("Points on the cricle")

#generate randoin 2d points 
pts = np.random.uniform(a, b, (int(n),2))
inside_pts = pts[np.sqrt(pts[:,0]**2+pts[:,1]**2)<=radius]
outside_pts = pts[np.sqrt(pts[:,0]**2+pts[:,1]**2)>radius]

fig, ax = plt.subplots(1)
ax.scatter(inside_pts[:,0], inside_pts[:,1], c='b', alpha=0.8, edgecolor=None,label="inside")
ax.scatter(outside_pts[:,0], outside_pts[:,1], c='r', alpha=0.8, edgecolor=None,label="outside")
#area = (inside_x.shape[0]/n)*((b-a)*(b-a))
value_of_pi = 2*(y_bar/radius**2) #Multiply by two since we caluclated the area of half circle in integrand
print("Vlaue of pi: ",2*value_of_pi)  
plt.legend()
plt.show()



# Example 01 (find the value of pi) 
# r = 1

# a = -r
# b = r
# x = np.random.uniform(low=a, high=b, size=100)

# # area of circle is integration of half circle multiplied by two 
# # it can be written also as integration of quarter of a circle multiplied by four as in lecture  (limits will be from 0 to r)
# def integrand(x):
#     return  2*np.sqrt(b-x**2)

# # Monte Carlo integration
# y = integrand(x)
# y_bar = (b-a)/ x.shape[0] * y.sum()

# #Actual Integration 
# I = quad(integrand, a, b,)
# print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
# plt.scatter(x, y)
# plt.show()