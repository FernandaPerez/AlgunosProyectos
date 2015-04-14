import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy import integrate, special

#Condiciones Iniciales
R0 = 1.
V0 = 5.

#Definimos las funciones especiales que necesitaremos
def Plm (grado, orden, x):
    return special.lpmv(orden, grado, x)
def Ylm (grado, orden, th, phi):
    return special.sph_harm(orden, grado, phi, th)

#Ahora los coeficientes Alm:
def Alm (grado, orden):
    integral = integrate.quad(Plm, -1., 1., args = (grado, orden))
    return np.sqrt(5. / (np.pi * 4. * np.factorial(4))) * (1./(R0 ** grado)) * integral

print "Todo bien"
