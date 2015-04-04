import numpy as np
from matplotlib import pyplot as plt
import scipy as spy
import scipy.integrate as integrate
from types import *
get_ipython().magic('matplotlib inline')



a = 1.   #Radio de la Esfera
V0 = 5.  #Potencial eléctrico V0



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
    


C = range(1,11)
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
            print (D, "  l = ", i, " m = ", m, "  Error = ", Clm(i,m)[1], "Mlm = ", Clm(i,m)[2] )
        print("  \n")




