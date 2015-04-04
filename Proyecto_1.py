
# coding: utf-8

# Una vez resuelta la ecuación de Laplace en coordenadas esféricas, por el método de separación de variables:
# $$\nabla ^2 V(r, \theta, \phi) = 0$$
# se obtienen los siguientes modos normales:
# $$ V_{lm}(r, \theta, \phi) = \left( D_{lm} r^l + E_{lm} \frac{1}{r ^{-(l+1)}} \right) Y_l ^m(\theta, \phi) $$
# Escritos también de la forma:
# $$ V_{lm}(r, \theta, \phi) = \left( D_{lm} r^l + E_{lm} \frac{1}{r ^{-(l+1)}} \right) e^{im\phi} P_l ^m (cos \theta). $$
# 
# Una vez evaluada las condiciones de frontera, es posible corroborar qué:
# $$ V(r, \theta, \phi) = \sum_ {l= 0} ^ \infty \sum_{-l} ^{l} \left( D_{lm} r^l \right) e^{im\phi} P_l ^m (cos \theta), ~~ l + m = 2k + 1 $$
# Donde la solución divergente en el origen ha sido removida para garantizar que el potencial sea finito en el punto (0,0,0) contenido dentro del plano z = 0. La condición en la frontera de $z = 0 \rightarrow V(r, \theta, \phi) = 0$, restringe los posibles valores de $l$ y $m$ aquellos tales que $l + m = 2k + 1$.
# 
# Imponiendo la siguiente condición a la frontera,
# $$ \sum_ {l= 0} ^ \infty \sum_{-l} ^{l} \left( D_{lm} a^l \right)  Y_l ^m(\theta, \phi) = f(\theta, \phi) $$
# Donde:
# $$ f(\theta, \phi) = \left\{ \begin{array}{cc}
# V_0 sin{\frac{\phi}{2}} ~ cos^2 \theta ~ sin{\frac{\phi}{2}} & 0 \leq \theta \leq \frac{\pi}{2} \\
# 0 & \frac{\pi}{2} \leq \theta \leq \pi
# \end{array}  \right. $$
# Multiplicando por $\hat{Y}_{l'} ^{m'}$:
# $$ \sum_ {l= 0} ^ \infty \sum_{-l} ^{l} \left( D_{lm} a^l \right)  \hat{Y}_{l'} ^{m'} Y_l ^m = f(\theta, \phi) ~\hat{Y}_{l'} ^{m'}$$
# Integrando:
# $$ \sum_ {l= 0} ^ \infty \sum_{-l} ^{l} \left( D_{lm} a^l \right) \int_0 ^\pi \int_0 ^{2\pi}  \hat{Y}_{l'} ^{m'} Y_l ^m ~sin\theta d\theta d\phi= V_0 \int_0 ^{\frac{\pi}{2}} \int_0 ^{2\pi} sin{\frac{\phi}{2}} ~ cos^2 \theta ~\hat{Y}_{l'} ^{m'} ~ sin\theta d\theta d\phi + \int_\frac{\pi}{2} ^\pi \int_0 ^{2\pi} 0 ~sin\theta d\theta d\phi$$
# Como:
# $$ \int_0 ^\pi \int_0 ^{2\pi}\hat{Y}_{l'} ^{m'} Y_l ^m ~sin\theta d\theta d\phi = \delta_{l, l'} \delta_{m, m'} $$
# 
# $$ D_{lm} a^l = V_0 \int_0 ^{\frac{\pi}{2}} \int_0 ^{2\pi} sin{\frac{\phi}{2}} ~ cos^2 \theta ~\hat{Y}_{l'} ^{m'} ~ sin\theta d\theta d\phi $$
# 
# Y siendo $\hat{Y}_l ^m (\theta, \phi) = N ~e^{-im\phi} P_l ^m (cos \theta)$, con $N = (-1)^m \sqrt{\frac{(2l + 1)(l-m)!}{4\pi (l+m)!}} $;
# 
# $$ D_{lm} a^l = V_0 N \int_0 ^{\frac{\pi}{2}} \int_0 ^{2\pi} sin{\frac{\phi}{2}} ~ cos^2 \theta ~e^{-im\phi} P_l ^m (cos \theta) ~ sin\theta d\theta d\phi $$
# Separando las componentes en $\theta$ y $\phi$:
# $$ D_{lm} = \frac{V_0 N}{a^l} \left[ \int_0 ^{\frac{\pi}{2}} cos^2 \theta ~sin\theta ~P_l ^m (cos \theta) d\theta \right] \left[\int_0 ^{2\pi}sin{\frac{\phi}{2}} ~ e^{-im\phi}   d\phi \right]     $$
# 
# Con el cambio de variable: $x = cos\theta$, $- sin \theta d\theta = dx$.
# \begin{equation}
#  D_{lm} = \frac{V_0 N}{a^l} \left[ -\int_1 ^0 x^2 ~P_l ^m (x) dx \right] \left[\int_0 ^{2\pi}sin{\frac{\phi}{2}} ~ e^{-im\phi}   d\phi \right] 
# \end{equation}
# Concentrándonos en la segunda integral, recordando que $sin \alpha = \frac{e^{i \alpha} - e ^{-i \alpha}}{2i} $;
# $$
# \int sin{\frac{\phi}{2}} ~ e^{-im\phi}   d\phi   = \int \left( \frac{e^{i \frac{\phi}{2}} - e^{-i \frac{\phi}{2}}      }{2i} \right) e ^{-i m \phi} d\phi $$
# 
# $$ = \frac{1}{2i} \int \left(    e^{i\frac{\phi}{2}} e ^{-im\phi} - e^{-i \frac{\phi}{2}} e ^{-im\phi} \right)d\phi 
#  $$
# 
# $$ = \frac{1}{2i} \int \left(  e ^{i \phi \left( \frac{1}{2} - m  \right) } - e ^{- i \phi \left( \frac{1}{2} + m  \right)} \right)  d\phi $$
# Separando la integral y efectuando los cambios de variable pertinentes:
# 
# $$ = \left. \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) } e ^{- i \phi \left( \frac{1}{2} + m  \right)}  - \frac{1}{ \left( \frac{1}{2} - m  \right) } e ^{i \phi \left( \frac{1}{2} - m  \right)}  \right\} \right|_0 ^{2\pi}$$
# 
# $$ = \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) } e ^{- i \pi \left( 1 + 2m  \right)}  - \frac{1}{ \left( \frac{1}{2} - m  \right) } e ^{i \pi \left( 1 - 2m  \right)}  \right\}  - \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) }  - \frac{1}{ \left( \frac{1}{2} - m  \right) }   \right\}$$
# 
# Usando la fórmula de Euler:
# 
# $$e ^{i  \pi \left( 1 +2 m  \right) }  = cos ((2m +1) \pi) + i ~sin ((2m+1) \pi)$$
# 
# Sabemos que $m$ es un entero, por lo tanto $(2m+1)\pi$ es un múltiplo entero \emph{impar} de $\pi$ y $sin ((2m+1) \pi) = 0, \forall m$. Con el mismo argumento, $cos((2m+1)\pi) = -1$. 
# 
# Análogamente:
# $$e ^{i  \pi \left( 1 -2 m  \right) }  = cos ((1 - 2m) \pi) - i ~sin ((1 - 2m) \pi)$$
# $$ = cos((2m-1) \pi) + i ~sin ((2m-1)\pi)) $$
# Y por razones idénticas a las expuestas con anterioridad: $sin((2m-1)\pi) = 0$ y $cos((2m-1)\pi) = -1$, $\forall m$.
# 
# Hecho esto:
# $$ = \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) } (-1)  - \frac{1}{ \left( \frac{1}{2} - m  \right) } (-1)  \right\}  - \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) }  - \frac{1}{ \left( \frac{1}{2} - m  \right) }   \right\}$$
# $$ =  - \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) }   - \frac{1}{ \left( \frac{1}{2} - m  \right) }  \right\}  - \frac{1}{2} \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) }  - \frac{1}{ \left( \frac{1}{2} - m  \right) }   \right\}$$
# $$ = -  \left\{  \frac{1}{ \left( \frac{1}{2} + m  \right) }   - \frac{1}{ \left( \frac{1}{2} - m  \right) }  \right\} $$
# 
# Y la expresión para los $D_{lm}$ se convierte en:
# $$  D_{lm} = \left\{ (-1)^m \sqrt{\frac{(2l + 1)(l-m)!}{4\pi (l+m)!}} \frac{V_0}{a^l} \left[\frac{1}{ \left( \frac{1}{2} - m  \right) } -  \frac{1}{ \left( \frac{1}{2} + m  \right) }    \right] \right\}  \int_0 ^1 x^2 ~P_l ^m (x) dx    $$
# 
# Definiendo $M_{lm}$:
# $$ M_{lm} = (-1)^m \sqrt{\frac{(2l + 1)(l-m)!}{4\pi (l+m)!}} \frac{V_0}{a^l} \left[\frac{1}{ \left( \frac{1}{2} - m  \right) } -  \frac{1}{ \left( \frac{1}{2} + m  \right) }    \right] $$
# 
# Es necesario notar que, para valores de $m = 0 $, los coeficientes $M_{lm} = 0$.

# In[13]:

import numpy as np
from matplotlib import pyplot as plt
import scipy as spy
import scipy.integrate as integrate
from types import *


# Introducimos los valores iniciales del problema. Por simplicidad, eligiremos una esfera unitaria y un potencial eléctrico de 5 volts.

# In[5]:

a = 1.   #Radio de la Esfera
V0 = 5.  #Potencial eléctrico V0


# Definimos una clase que permitirá efectuar operaciones con el grado y el orden de los polinomios con mayor eficiacia.

# In[6]:

class Grado_Pol:
    
    def __init__ (self, l):
        if float(l).is_integer():                     # Comprueba que el grado sea un entero
            self.Grado = l
        else:
            print("El grado es invalido")
    
    def Orden (self):
        lista = []
        Emes = np.arange(-self.Grado, self.Grado +1)
        for m in Emes:
            if (self.Grado + m)%2 != 0:               # Elige unicamente los valores de m que satisfacen l+m = impar
                lista.append(m)
                
        return lista                                 # Devuelve una lista de valores posibles
    
    def No_Orden (self):
        return len(self.Orden())                      # Indica la cardinalidad de la lista anterior
        
        
        
        


# Definimos los coeficientes $D_{lm}$.

# In[7]:

def Clm (l, m):                                  # El coeficiente Clm es un auxiliar, permite conocer informacion util
    
    def Legendre (x, grado, orden):
        return spy.special.lpmv(orden, grado, x)
    def Mlm (l, m):
        return ((-1.)**m)*np.sqrt(((2.*l + 1)*np.math.factorial(l-m))/(4*np.pi*np.math.factorial(l+m)))*(V0 / (a**l))*((1. / (0.5 - m)) - (1. / (0.5 + m)))
    integral = integrate.quad(Legendre, 1., 0., args=(l,m))
    
    return Mlm(l,m)*integral[0], integral[1], Mlm(l,m)
    # Clm[0] = el coeficiente buscado
    # Clm[1] = error del metodo quad de scipy
    # Clm[2] = Valor de Mlm, para ver si efectivamente es cero.

def Dlm (l,m):
    return Clm(l,m)[0]
    


# In[8]:

def Ylm (r, th, phi, grado, orden):
    return spy.special.sph_harm(orden, grado, phi, th)
    


# In[22]:

Rango = 4

def V(r, th, phi, Rango=3):
    V = 0.
    for i in range (0,4):
        l = Grado_Pol(i)
        if not l.Orden():
            print("El grado ", l.Grado, " no es valido")
        else:
            print("Para el grado ", i, " hay ", l.No_Orden(), " ordenes.")
            for m in l.Orden():
                D = Dlm(i,m)
                if D < 1e-6 :
                    D = 0
                V = V + D*Ylm(r, th,phi, i, m)
                print (D, "  l = ", i, " m = ", m, "  Error = ", Clm(i,m)[1], "Mlm = ", Clm(i,m)[2] )
            print("  \n")
    
    return V

    


# In[23]:

rr = np.linspace(0, a, 5)
tt = np.linspace(0, np.pi, 5)
pp = np.linspace(0, 2*(np.pi), 5)


# In[24]:

R,Th,Ph = np.meshgrid(rr, tt, pp)


# In[ ]:



