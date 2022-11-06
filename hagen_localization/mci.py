import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad


# # Example 01 (General case)
# a = 0
# b = 1
# x = np.random.uniform(low=a, high=b, size=100)

# def integrand(x):
#     return  4*np.sqrt(1-x**2)

# # Monte Carlo integration
# y = integrand(x)
# y_bar = (b-a)/ x.shape[0] * y.sum()

# #Actual Integration 
# I = quad(integrand, a, b,)
# print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
# plt.scatter(x, y)
# plt.show()




# Example 01 (find the value of pi) 
r = 1

a = -r
b = r
x = np.random.uniform(low=a, high=b, size=100)

# area of circle is integration of half circle multiplied by two 
# it can be written also as integration of quarter of a circle multiplied by four as in lecture  (limits will be from 0 to r)
def integrand(x):
    return  2*np.sqrt(b-x**2)

# Monte Carlo integration
y = integrand(x)
y_bar = (b-a)/ x.shape[0] * y.sum()

#Actual Integration 
I = quad(integrand, a, b,)
print("Actual integration: ", I[0], "Monte Carlo integration: ", y_bar)
plt.scatter(x, y)
plt.show()