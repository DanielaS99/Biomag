from sympy import elliptic_e, elliptic_k
import numpy as np
from math import pi, sqrt

u0 = 4 * pi * 10 ** -7  #Air's relative magnetic permeability
Bmax = 2.16     #Magnetic flux density saturation
ur = 200    #Core's relative magnetic permeability
I=10  #Currente

def Bx(x, y, z, magPerArea, a):
    if x == 0:
        return 0
    C = u0 * magPerArea / pi
    p = sqrt(x ** 2 + y ** 2)
    r2 = x ** 2 + y ** 2 + z ** 2
    alfa2 = a ** 2 + r2 - 2 * a * p
    beta = sqrt(a ** 2 + r2 + 2 * a * p)
    k2 = 1 - alfa2 / beta ** 2

    return C * x * z / (2 * alfa2 * beta * p ** 2) * ((a ** 2 + r2) * elliptic_e(k2) - alfa2 * elliptic_k(k2))


def By(x, y, z, magPerArea, a):
    if y == 0:
        return 0
    C = u0 * magPerArea / pi
    p = sqrt(x ** 2 + y ** 2)
    r2 = x ** 2 + y ** 2 + z ** 2
    alfa2 = a ** 2 + r2 - 2 * a * p
    beta = sqrt(a ** 2 + r2 + 2 * a * p)
    k2 = 1 - alfa2 / beta ** 2

    return C * y * z / (2 * alfa2 * beta * p ** 2) * ((a ** 2 + r2) * elliptic_e(k2) - alfa2 * elliptic_k(k2))


def Bz(x, y, z, magPerArea, a):
    C = u0 * magPerArea / pi
    if x == 0 and y == 0:
        r2 = z ** 2 # z²
        alfa = sqrt(a ** 2 + r2) #z
        return C / (2 * alfa**3) * ((a ** 2 - r2) * pi/2 + alfa**2 * pi/2)
    #          u0 * magPerArea / (2 * a)

    p = sqrt(x ** 2 + y ** 2)
    r2 = x ** 2 + y ** 2 + z ** 2
    alfa2 = a ** 2 + r2 - 2 * a * p
    beta = sqrt(a ** 2 + r2 + 2 * a * p)
    k2 = 1 - alfa2 / beta ** 2
    return C / (2 * alfa2 * beta) * ((a ** 2 - r2) * elliptic_e(k2) + alfa2 * elliptic_k(k2))


def Bxy(x, y, z, magPerArea, a):
    Bx1 = Bx(x, y, z, magPerArea, a)
    if x != 0:
        By1 = Bx1 * (y / x)
    else:
        By1 = By(x, y, z, magPerArea, a)

    return Bx1, By1


def axisCoreField(a, L, N, I, Zmax, res):
  Xm = ur - 1
  # H = I*N/L
  # M = Xm*H
  # magPerArea es mag/A, que es M*V/A, que es M*L, que es Xm*H*L, que es Xm*I*N/L*L, que es Xm*I*N
  magPerAreaTest = Xm*I*N
  magPerAreaMax = Bmax/u0*(2*a)

  magPerArea = min(magPerAreaTest, magPerAreaMax)
  sat = magPerAreaTest >= magPerAreaMax
  print('magTest:{:.2f}, magFinal:{:.2f}'.format(magPerAreaTest, magPerArea))

  # Saturation scenario is chosen only if M*L is higher
  #print(magPerAreaMax, magPerAreaTest, magPerArea)

  zs = []
  Bs = []
  for i in range(0, res):
    z = i*Zmax/res + L/2
    B = Bz(0, 0, z, magPerArea, a) #u0*magPerArea/2 * (a**2/(z**2 + a**2)**1.5)

    zs.append(z*100)
    Bs.append(B)

  return zs, Bs, sat


calibres = [28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14] #wire gauge number AWG
#Diameter of the corresponding AWG wire gauge
dCables = [0.00032, 0.00036, 0.00040, 0.00045, 0.00051, 0.00057, 0.00064, 0.00072, 0.00081, 0.00091, 0.00102, 0.00115, 0.00129, 0.00145, 0.00163]
#Maximum current supported by each gauge
iCablesMax = [4, 4.5, 5, 6, 7, 8, 9, 10.5, 12, 14, 16, 18, 20, 22.5, 25] #rated 90C
#Current experimented by the cable under the conditions stated
iCables = [0.09279926, 0.14806821, 0.23286923, 0.37082112, 0.5808424, 0.93579198,1.47289915, 2.34457353, 10, 5.73356228, 9.40068738,  14.81414763, 22.99519814, 37.3374291, 59.42115241]
#Wire resistance per pound
RCables = [212.87, 168.82, 133.86, 106.17, 84.20, 66.78, 52.93, 41.98, 33.29, 26.41, 20.94, 16.61, 13.17, 10.44, 8.282]
#Wire length per pound
LCables = [1993*5, 1575*5, 1263*5, 1000*5, 805*5, 630*5, 2525, 2000, 1600, 1300, 1000, 800, 650, 505, 400]

import numpy as np
RCables = np.array(RCables)
LCables = np.array(LCables)
ResistanceCables = RCables/1000*LCables*0.3048/5
CorrienteCables = 12/ResistanceCables
matriz = np.transpose([calibres, CorrienteCables, iCablesMax])
#print(np.array(['Calibre', 'Corriente a 12V', 'Corriente máxima']))
#print(matriz)

def axisCoilField(a, ratioZR, calibre, divisor, Zmax, res):
  iC = calibres.index(calibre)
  #I = iCables[iC]
  dCable = dCables[iC]
  R = RCables[iC]
  length = LCables[iC]*0.3048/divisor #De pies a metros

  cableUsado = 0
  Nz = 0
  Nr = 0
  while cableUsado < length:
    Nr += 1
    cableUsado += (a + dCable*(Nr-0.5))*2*pi * Nz #El radio es "a + dCable*(Nr-1) + dCable/2", pero se simplifica "a + dCable*(Nr-0.5)"
    aumentoEnZ = int(Nr*ratioZR - Nz)
    Nz += aumentoEnZ
    #El cable añadido debido al aumentoEnZ es la suma finita de (a + dCable*(i-0.5)) donde i va de 1 hasta Nr multiplicado por 2*pi*aumentoEnZ
    #Esta suma es (a - 0.5*dCable)*Nr + dCable*Nr*(Nr+1)/2 o a*Nr + dCable*Nr**2/2, entonces:
    cableUsado += (a*Nr + dCable*Nr**2/2)*2*pi*aumentoEnZ

  Nzf = Nz
  while cableUsado > length and Nzf > 0:
      cableUsado -= (a + dCable*(Nr-0.5))*2*pi
      Nzf -= 1

  #Nz = 5
  #Nzf = 5
  #Nr = 5
  #print(cableUsado, length, Nzf)

  Bs = []
  zs = []
  for i in range(0, res):
    z = i*Zmax/res
    B=0
    for j in range(Nr):
      ai = a + dCable*j
      if j == Nr-1:
        Nzj = Nzf
      else:
        Nzj = Nz
      for k in range(Nzj):
        zi = z + dCable*k
        B += Bz(0, 0, zi, I, ai) #u0*I/2 * (ai**2/(zi**2 + ai**2)**1.5)

    Bs.append(B)
    zs.append(z*100)

  return (zs, Bs, I, R, cableUsado, Nr, Nz, Nzf)

def plotAxisTotalFields(a, Zmax, res, ratioZR, divisor, calibre):
    import matplotlib.pyplot as plt
    import numpy as np

    plt.xlabel("z (cm)")
    plt.ylabel("B (T)")
    plt.ylim(0, 2.5)
    plt.grid()

    legends = []

    #for calibre in calibres_:
    print(calibre)
    i = calibres.index(calibre)
    zs, Bs, I, R, length, Nr, Nz, Nzf = axisCoilField(a, ratioZR, calibre, divisor, Zmax, res)
    zs, Bcs, sat = axisCoreField(a, Nz * dCables[i], (Nr-1) * Nz + Nzf, I, Zmax, res)

    print('a: {:.2f},  Calibre: {},  Nz: {},  Nr: {}, I: {}, L: {}, V: {}'.format(a, calibre, Nz, Nr, I, Nz*dCables[i], I*length*R/1000))

    if sat:
        plt.plot(np.array(zs), np.array(Bs) + np.array(Bcs), '--')
    else:
        plt.plot(zs, np.array(Bs) + np.array(Bcs))

    legends.append('AWG={} V={:.2f} I={:.1f}'.format(calibre, I * R * length / 1000, I))

    #plt.title('Bobinas con distintos radios ({:.2f} lb) ({} A)'.format(5/divisor, I))
    #plt.title('Bobinas con distintos radios 5x5 vueltas (10 A)'.format(5 / divisor))
    plt.title('({:.2f} lb) ({} A) (ur= {}) ({} AWG)'.format(5/divisor, I,ur, calibre))
    #plt.title('({:.2f} lb) ({} A) (ur= {})'.format(5 / divisor, I, ur))
    plt.legend(legends)
    #plt.show()

def coreFieldMap(a, L, N, I, x0, y0, z, Xlim, Ylim, res):
    Xm = ur - 1
    H = I*N/L
    M = Xm*H
    # magPerArea es mag/A, que es M*V/A, que es M*L, que es Xm*H*L, que es Xm*I*N/L*L, que es Xm*I*N
    magPerAreaTest = Xm * I * N
    magPerAreaMax = Bmax / u0 * (2 * a)

    magPerArea = min(abs(magPerAreaTest), abs(magPerAreaMax))  # se escoge la saturada solo si M*L es mayor
    magPerArea *= abs(magPerAreaTest)/magPerAreaTest # se mantiene el signo orignial

    print('magTest:{}, magFinal:{}'.format(magPerAreaTest, magPerArea))

    from numpy import zeros, array
    B = zeros((res, res, 3))

    for i in range(res):
        x = Xlim[0] + i * (Xlim[1]-Xlim[0]) / res
        for j in range(res):
            y = Ylim[0] + j * (Ylim[1] - Ylim[0]) / res
            Bx, By = Bxy(x-x0, y-y0, z+L/2, magPerArea, a)
            Bz1 = Bz(x-x0, y-y0, z+L/2, magPerArea, a)
            B[j, i] = array([Bx, By, Bz1])

    return B

def saveCoreFieldMap(name, a, L, N, I, x0, y0, z, Xlim, Ylim, res):
    np.save(name, coreFieldMap(a, L, N, I, x0, y0, z, Xlim, Ylim, res))

pesoMetro=[]
for long in LCables:
    pesoCalibre=5/(long*0.3048)
    pesoMetro.append(pesoCalibre)

arrPesoMetro=np.array(pesoMetro)
arrResist= np.array(RCables)


def cantidadCable(a, N, I):
    NI= N*I
    longT= 2*pi*a*N
    pesoT= arrPesoMetro*longT
    resisT= arrResist*longT/1000
    VoltT= resisT*I
    matrizC = np.transpose([calibres, iCablesMax, pesoT.tolist(), VoltT.tolist()])
    print("N*I = ", NI)
    print("Longitud = {:.2f} m".format(longT))
    print("Radio = {} cm".format(a*100))
    print("Corriente = {} A".format(I))
    print(np.array(['Calibre', 'Corriente Max', 'Peso', 'Voltaje']))
    float_formatter = "{:.3f}".format
    np.set_printoptions(formatter={'float_kind': float_formatter})
    print(matrizC)

import matplotlib.pyplot as plt

def voltajeNecesario(divisor):
    print("Corriente = {} A".format(I))
    print("Peso = {:.2f} lb ({:.0f} bobinas)".format(5/divisor, divisor/5))
    for iC in range(15):
        longitud=LCables[iC]*0.3048/divisor
        resistencia=longitud*RCables[iC]/1000
        voltaje=resistencia*I
        print('{} AWG -> {:.2f} V'.format(calibres[iC],voltaje))


def efectoJoule(I, V, t, m, ce, Ti):
    P=I*V
    Q=P*t
    deltaT= Q/(m*ce)
    Tf=deltaT+Ti
    print("Tf= ", Tf)
