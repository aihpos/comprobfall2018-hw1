import numpy as np
import matplotlib.pyplot as plt


# width and height of grid
w = 10
h = 10

# generates 2d matrix size[w][h]
gridplot = [[0 for x in range(w)] for y in range(h)]

gridplot[0][0] = 1
gridplot[1][1] = 1
gridplot[2][2] = 1
gridplot[3][3] = 1
gridplot[4][4] = 1
gridplot[5][5] = 1
gridplot[6][6] = 1
gridplot[7][7] = 1
gridplot[8][8] = 1
gridplot[9][9] = 1

plt.matshow(gridplot)
plt.show()