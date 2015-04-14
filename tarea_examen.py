import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from scipy import integrate, special
from matplotlib import cm, colors

#Condiciones Iniciales
R0 = 1.
V0 = 5.

#Definimosx las funciones especiales que necesitaremos
def Plm (x, grado, orden):
    return special.lpmv(orden, grado, x)
def Ylm (grado, orden, th, phi):
    return special.sph_harm(orden, grado, phi, th)
#Ahora los coeficientes Alm:
def Alm (grado, orden):
    integral = integrate.quad(Plm, 1., -1., args = (grado, orden))
    if integral <= 1e-8:
        integral = 0
    a =  np.sqrt(5. / (np.pi * 4. * np.math.factorial(4))) * (1./(R0 ** grado)) * integral[0]*complex(0, (8*V0 / orden))

    print a
    return a
#Grados del polinomio que estamos dispuestos a sumar
M = 4

#Iteramos, seleccionando los ordenes validos
def V (r, th, phi, M=10):
    
    V = 0.
    for l in range (1, M+1):
        lista = []
        for m in range(-l, l+1):
            if m % 4 == 0 and (m/4) % 2 != 0:
                lista.append(m)

        for m_valida in lista:
            V =  V + Alm (l, m_valida)*Ylm(l, m_valida, th, phi)
    return V
        
pasos = 100

th = np.linspace(0. , 2*np.pi, pasos)
rr = np.linspace(0., R0, pasos)

R, Th = np.meshgrid(rr, th)
Ph = np.ones_like(R) * (np.pi / 4.)
X = R*np.cos(Th)
Y = R*np.sin(Th)

color_map= np.real(V(R,Th,Ph))

fig = plt.figure()
ax = plt.axes()
ax.set_title(r"Grafica para $V(r, \theta, \phi = \frac{\pi}{2})$")
im = ax.pcolor(X, Y,color_map)
plt.colorbar(im, shrink=0.7)

fig.savefig("potencial.png")
plt.show()
    

print "Todo bien"
