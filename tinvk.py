import math


def invK(x, y, z):
    k2 = 180.0 / math.pi
    l0 = math.sqrt(x * x + y * y + z * z)
    theta = math.asin(l0 / 2.0 / 120.0) * k2
    # print("theta:", theta)

    alpha = 90.0 - theta
    # print("alpha:", alpha)
    if z >= 0:
        beta = math.asin(z / l0) * k2
        j1 = 90.0 - (alpha + beta)
    else:
        beta = math.asin(-z / l0) * k2
        j1 = 90.0 - (alpha - beta)

    j2 = 90.0 - 2 * theta + j1
    # print("j1:", j1)
    # print("j2:", j2)

    l1 = math.sqrt(x * x + y * y)
    if y >= 0:
        j0 = math.asin(y / l1) * k2
    else:
        j0 = -math.asin(-y / l1) * k2
    return j0, j1, j2

zz = 60*1.414
print(zz)
print(invK(60, 0, -zz))