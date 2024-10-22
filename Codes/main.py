import multiprocessing
import numpy as np

if __name__ == '__main__':
    from field import plotAxisTotalFields, calibres, coreFieldMap, saveCoreFieldMap, voltajeNecesario, efectoJoule
    import matplotlib.pyplot as plt
    legends = []

    #a=0.05
    for i in range(10):
        a = i*0.01+0.01
        plotAxisTotalFields(a, 0.1, 200, 1, 15, calibres[8])
        legends.append('a= '+'{:.0f}'.format(a*100)+' cm')
    plt.legend(legends)
    plt.show()

    res = 100
    #sep = 0.07
    sep=0.00081*15+0.05 #distance or separation between the origin and the center of each electromagnet, considering the wire diameter
    a = 0.05    #core's radius
    N = 300     #Number of turns
    I = 10/3      #Current
    L = 0.04    #Electromagnet's length
    Xlim = [-0.1, 0.1]
    Ylim = [-0.1, 0.1]
    z = 0.1


    from math import cos, sin, pi


    B = np.zeros((res, res, 3))
    procecess = []

    #Extra code for arrays with a central electromagnet
    # p = multiprocessing.Process(target=saveCoreFieldMap, args=('field0', a, L, N, I, 0.0000001, 0.0000001, z, Xlim, Ylim, res))
    # p.start()
    # procecess.append(p)
    for i in range(3):
        x0=sep*np.cos(np.pi/2 + 2*i*np.pi/3) #electromagnet's center (these formulas change according to the geometry of the array)
        y0=sep*np.sin(np.pi/2 + 2*i*np.pi/3)
        print("x0 y0")
        print(x0)
        print(y0)
        p = multiprocessing.Process(target=saveCoreFieldMap, args=('field'+str(i), a, L, N, I, x0, y0, z, Xlim, Ylim, res))
        p.start()
        procecess.append(p)

    for i in range(len(procecess)):
        procecess[i].join()
        B += np.load('field' + str(i) + '.npy')

    import matplotlib.pyplot as plt
    import numpy as np

    import matplotlib as mpl
    from matplotlib.colors import LinearSegmentedColormap, ListedColormap

    viridis = mpl.colormaps['magma']

    import numpy as np
    data = np.sqrt(B[:, :, 0] ** 2 + B[:, :, 1] ** 2 + B[:, :, 2] ** 2)

    print(data.shape)
    print(B[:, :, 2].shape)
    n = 1
    fig, axs = plt.subplots(1, n, figsize=(n * 2 + 2, 3), layout='constrained', squeeze=False)
    for [ax, cmap] in zip(axs.flat, [viridis]):
        X = np.array(range(res))*20/res-10
        Y = X
        psm = ax.pcolormesh(X, Y, data, cmap=cmap, antialiased=True, rasterized=True, vmin=data.min(), vmax=data.max())
        cbar= fig.colorbar(psm, ax=ax)

    circle = plt.Circle((0, 0), 16/2, fill=False, label="Container r=8cm",)
    ax = plt.gca()
    ax.add_patch(circle)
    ax.legend( fontsize= 8)
    cbar.ax.set_title('B (T)', fontsize= 8)
    ax.set_title("z = {:.0f}cm".format(z*100))
    ax.set_xlabel("X (cm)", fontsize= 8)
    ax.set_ylabel("Y (cm)", fontsize= 8)
    plt.show()

    from field import cantidadCable
    cantidadCable(0.05, 125, 8)

    # legends1 = []
    # for i in range(15):
    #     plotAxisTotalFields(0.05, 0.1, 200, 1, 15, calibres[i])
    #     legends1.append('{} AWG'.format(calibres[i]))
    # plt.legend(legends1)
    # plt.show()

    #voltajeNecesario(10)

    #efectoJoule(25/3,10,300,454*3,0.39,25)

