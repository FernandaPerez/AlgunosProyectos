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

#Grados del polinomio que estamos dispuestos a sumar
M = 4

#Iteramos, seleccionando los ordenes validos
def V (r, th, phi, M=4):
    
    V = 0.
    for l in range (1, M+1):
        lista = []
        for m in range(-l, l+1):
            if m % 4 == 0 and (m/4) % 2 != 0:
                lista.append(m)

        for m_valida in lista:
            V = V + Alm (l, m_valida)*(1./(r**grado))*Ylm(l, m_valida, th, phi)
    return V
        
pasos = 100

th = np.linspace(0. , np.pi, pasos)
rr = np.linspace(0., R0, pasos)

R, Th = np.meshgrid(rr, th)

X = R*np.cos(Th)
Y = R*np.sin(Th)

fig = plt.figure()
ax = plt.axes()
im = ax.scatter(X, Y)

plt.show()
    

print "Todo bien"
