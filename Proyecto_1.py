
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

# In[2]:

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm, colors
import scipy as spy
import scipy.integrate as integrate
from types import *
from mpl_toolkits.mplot3d import axes3d, Axes3D 
import matplotlib.cm as cm

#%matplotlib inline


# Introducimos los valores iniciales del problema. Por simplicidad, eligiremos una esfera unitaria y un potencial eléctrico de 5 volts.

# In[3]:

a = 1.   #Radio de la Esfera
V0 = 5.  #Potencial eléctrico V0


# Definimos una clase que permitirá efectuar operaciones con el grado y el orden de los polinomios con mayor eficiacia.

# In[4]:

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
        
        
        
        


# A continuación, las funciones especiales que serán necesarias: Los polinomios asociados de legendre y los armonicos esferico, de los que únicamente será posible retomar la parte real.

# In[5]:

def Legendre (x, grado, orden):
        return spy.special.lpmv(orden, grado, x)
def Ylm (r, th, phi, grado, orden):
    return spy.special.sph_harm(orden, grado, phi, th)


# Definimos los coeficientes $D_{lm}$.
# La integral $\int_0 ^1 x^2 ~P_l ^m (x) dx $ será realizada con la libreria scypy.integrate.quad

# In[6]:

def Clm (l, m):                                  # El coeficiente Clm es un auxiliar, permite conocer informacion util
    
    def Mlm (l, m):
        return ((-1.)**m)*np.sqrt(((2.*l + 1)*np.math.factorial(l-m))/(4*np.pi*np.math.factorial(l+m)))*(V0 / (a**l))*((1. / (0.5 - m)) - (1. / (0.5 + m)))
    integral = integrate.quad(Legendre, 1., 0., args=(l,m))
    
    return Mlm(l,m)*integral[0], integral[1], Mlm(l,m)
    # Clm[0] = el coeficiente buscado
    # Clm[1] = error del metodo quad de scipy
    # Clm[2] = Valor de Mlm, para ver si efectivamente es cero.

def Dlm (l,m):
    return Clm(l,m)[0]
    


# Para la funcion potencial, es necesario realizar la "suma infinita" sobre los grados $l$ del polinomio. Por simplicidad, la variable de grado a sido renombrada como *Rango*, y puede manipularse para tomar una *mejor aproximacion*. Como primera propuesta, para ahorrar recursos de computo, ha sido designada con un valor predeterminado de 4.

# In[34]:

def V(r, th, phi, Rango=4):
    V = 0.                           #Fuera del ciclo. Queremos sumar las todas las m's, sobre todas las i's.
    progreso = 0.
    for i in range (1,Rango+1):
        l = Grado_Pol(i)
        if not l.Orden():
            print("El grado ", l.Grado, " no es valido")
        else:
            #print("Para el grado ", i, " hay ", l.No_Orden(), " ordenes validos.")
            for m in l.Orden():
                D = Dlm(i,m)
                if D < 1e-6 :
                    D = 0
                V = V + D*Ylm(r, th,phi, i, m)
                #print (D, "  l = ", i, " m = ", m, "  Error = ", Clm(i,m)[1], "Mlm = ", Clm(i,m)[2] )
            progreso =  (i)*(100./Rango)
            #print ("... %d "% progreso,  "%")
    
                            
    return V

    


# La funcion a graficar es, entonces, una funcion de 4 variables: tres coordenadas espaciales (en coordenadas esfericas), y una cuarta coordenada que representa el valor del potencial $V(r, \theta, \phi)$. Para visualizar una imagen, la variable $r$ será fijada como parámetro. La informacion del potencial sera incluida como un mapa de colores por medio de las librerias matplotlib.cm y matplotlib.colors, e incorporada a una plot_surface de la función $f(r, \theta, \phi)$ como una máscara de color, para cada punto.
# 
# Considerando únicamente la componente real de V y mapeando los puntos para Th y Ph obtenidos (un arreglo de N por N. De incluir la variable r sería de N x N x N):

# In[35]:

"""
color_map= np.real(V(a,Th,Ph,5))
norm = colors.Normalize()
color = norm(color_map)

colorbar = cm.ScalarMappable(cmap = cm.jet)
colorbar.set_array(color_map)
"""


# Se hace necesario crear una malla de puntos para:
# $$ 0 \leq \theta \leq \pi/2 ~; ~ 0 \leq \phi \leq \pi$$

# In[36]:

#Estos espacios son necesarios para la creacion de una malla de C x C puntos
C = 100


# In[37]:

tt = np.linspace(0, (np.pi)/2. , C)
pp = np.linspace(0, 2*(np.pi), C)
        
Th,Ph = np.meshgrid( tt, pp)


# Incluyendo el mapa de colores:

# In[38]:

"""
** Esto tambien, necesito que corra para diferentes colores del radio

color_map= np.real(V(R,Th,Ph,5))
norm = colors.Normalize()
color = norm(color_map)

colorbar = cm.ScalarMappable(cmap = cm.jet)
colorbar.set_array(color_map)
"""


# In[53]:

def graficas (N=1, th=Th, phi=Ph, Rango=4, factor_ciclo=1):
    
    print("-----------------------------------------------------")
    contador = 0.
    fig = plt.figure(figsize=(6*N,N*6))
    for i in range(1,N+1):
    
        #print ("GRAFICA %d de %d" % (i, N))
        
        R = i*(a/(factor_ciclo*N))
        t = (25./N)
        s = "..."
        X = R*np.sin(Th)*np.cos(Ph)
        Y = R*np.sin(Th)*np.sin(Ph)
        Z = R*np.cos(Th)
        #print("(Malla creada para r = %3.3f)"%R)
        
        contador += t
        print(s , contador, "%")
        
        color_map= np.real(V(R,Th,Ph,Rango))
        norm = colors.Normalize()
        color = norm(color_map)
        
        contador += t
        print(s , contador, "%")

        colorbar = cm.ScalarMappable(cmap = cm.jet)
        colorbar.set_array(color_map)
        if (N%2) != 0:
            ax = fig.add_subplot(N,1, i, projection="3d")
        else:
            ax = fig.add_subplot(N/2,N/2, i, projection="3d")
        
        contador += t
        print(s , contador, "%")
        
        ax.set_xlim(-1,1)
        ax.set_ylim(-1,1)
        ax.set_zlim(-1,1)
        ax.set_title(r"Grafica de $ V(r, \theta, \phi)$ para $r = %2.2f $ (grado del polinomio $l = %d$)" % (R,Rango))
        ax.plot_surface(X, Y, Z,  rstride=1, cstride=1, facecolors=cm.jet(color))
        fig.colorbar(colorbar, shrink=0.7)
        
        contador += t
        print(s , contador, "%")   
    
    fig.savefig("%d_grado_%d.png"%(factor_ciclo, Rango))
    print ("Puntos graficados: ",Th[0].size)
    print("Grado maximo del polinimio: ", Rango)    
    print("-----------------------------------------------------")
    plt.clf()
    plt.cla()
    plt.close(fig)


# In[57]:

#graficas(Rango=2)


# In[56]:

v = input("No. de cuadros")

def cuadros (frames=10):
    for l in range(1,frames):
        graficas(Rango=6, factor_ciclo = l)
        print("--------------->   cuadro %d guardado"%l)


# In[55]:

cuadros(v)


# In[ ]:



