#!/usr/bin/env python
import matplotlib as mpl
from matplotlib import pyplot
import numpy as np
import pandas as pd 
import sys
import math

data = pd.read_csv(sys.argv[1],sep=',',header=None,engine='python')

df = data.to_numpy()

size_x = int(math.sqrt(df.shape[1]))
print(size_x)

zvals = np.zeros((size_x,size_x))

for i in range(size_x):
    for j in range(size_x):
        zvals[i][j] = df[0][j*size_x+i]
# make values from -5 to 5, for this example


# make a color map of fixed colors
cmap = mpl.colors.ListedColormap(['blue','black','red'])
bounds=[0,128,255]
norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

# tell imshow about color map so that only set colors are used
img = pyplot.imshow(zvals,interpolation='nearest',
                    cmap = cmap,norm=norm)

# make a color bar
pyplot.colorbar(img,cmap=cmap,
                norm=norm,boundaries=bounds,ticks=[0,128,255])

pyplot.show()
