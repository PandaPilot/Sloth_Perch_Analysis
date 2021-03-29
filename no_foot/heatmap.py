import seaborn as sns;# sns.set_theme()
import csv, math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import Axes3D
        
data=pd.read_csv('Results.csv')
data_plot=data[['Coarseness','OD','roll_i','I_f']]

od=data['OD'].unique()
coarse=data['Coarseness'].unique()
for i in range(0,len(od)):
    for j in range(0,len(coarse)):
        data=data.append(pd.Series([coarse[j],od[i],0,0,'-',0], index=data.columns), ignore_index=True)

data_vis=data.sort_values(by=['OD'])
data_c=data_vis[data_vis.Coarseness=='c']
R_i=data_c['roll_i'].unique()
X, Y = np.meshgrid(od, R_i)
I=np.zeros(X.shape)
for i in range(0,len(I)-1):
    for j in range(0,len(I[0])-1):
        pos_index=np.where(np.logical_and(data_c.OD==od[j],data_c.roll_i==R_i[i]))
        if len(pos_index[0])>0:
            I[i,j]=data_c.I_f.iloc[pos_index].to_numpy()
   
fig = plt.figure()
ax = fig.gca(projection='3d')
#hmap_c=sns.heatmap(X,Y,I)
surf = ax.plot_surface(X, Y, I, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

# Customize the z axis.
#ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
#with open('Results.csv', 'rt') as f1:
#        R = list(csv.reader(f1))

#data=np.array(R[1:])
#data_c=np.float_(data[np.where(data[:,0]=='c'),1:])
#data_m=np.float_(data[np.where(data[:,0]=='m'),1:])
#data_f=np.float_(data[np.where(data[:,0]=='f'),1:])
#data_n=np.float_(data[np.where(data[:,0]=='n'),1:])
#
#data_c=data_c[0,:,:]
#data_m=data_m[0,:,:]
#data_f=data_f[0,:,:]
#data_n=data_n[0,:,:]
#
#d_c_map=np.ones((len(data_c),len(data_c)))
#
#data_c_od=np.multiply(data_c[:,0],d_c_map)
#data_c_ang=np.multiply(data_c[:,1],d_c_map)
#data_c_cur=np.multiply(data_c[:,4],d_c_map)

#hmap_c=sns.heatmap(data_c_od,data_c_ang,data_c_cur)