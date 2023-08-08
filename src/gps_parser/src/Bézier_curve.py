import matplotlib.pyplot as plt

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

line = curva_b(290974.9485474154,3980000.3780540265, 290974.11204393406,3980007.7242259914, 290968.85334492486,3980011.369203906)
# print(line)

with open("curved.txt", "a") as f:
    for x, y in line:
        f.write(f"{x},{y}\n")