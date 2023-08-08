import numpy as np
from math import sqrt, pow, pi

from numpy.lib.index_tricks import AxisConcatenator

# 직선의 방정식을 활용한 포인트 계산
def get_linear_line(x1, y1, x2, y2):
    m = (y2-y1) / (x2-x1)
    n = y1 - (m * x1)

    x = None
    if x1 > x2:
        x = np.arange(x1, x2, -0.1)
    else:
        x = np.arange(x1, x2, 0.1)

    points = []
    for i in x:
        y = m*i + n
        points.append([i, y])

    return points

#############################
# 베지에 곡선을 활용한 포인트 계산
def recta(x1, y1, x2, y2):
    a = (y1 - y2) / (x1 - x2)
    b = y1 - a * x1
    return (a, b)

def curva_b(xa, ya, xb, yb, xc, yc, ran=30):
    (x1, y1, x2, y2) = (xa, ya, xb, yb)
    (a1, b1) = recta(xa, ya, xb, yb)
    (a2, b2) = recta(xb, yb, xc, yc)
    puntos = []

    for i in range(0, ran):
        if x1 == x2:
            continue
        else:
            (a, b) = recta(x1, y1, x2, y2)
        x = i*(x2 - x1)/ran + x1
        y = a*x + b
        puntos.append((x,y))
        x1 += (xb - xa)/ran
        y1 = a1*x1 + b1
        x2 += (xc - xb)/ran
        y2 = a2*x2 + b2
    return puntos
#############################

nodes = open("nodes.txt", "r").readlines()
final_point = []

for line in nodes:
    line = line.split(",")
    if len(line) == 4:
        points = get_linear_line(float(line[0]), float(line[1]), float(line[2]), float(line[3]))
        for i in points:
            final_point.append(i)
    elif len(line) == 6:
        points = curva_b(float(line[0]), float(line[1]), float(line[2]), float(line[3]), float(line[4]), float(line[5]))
        for i in points:
            final_point.append(i)

for p in final_point:
    with open("path.txt", "a") as f:
        f.write(f"{p[0]},{p[1]}\n")
