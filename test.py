import numpy as np
from math import radians, degrees, cos, sin, asin, fabs, atan2


def _clamp(value, x, y):
    n = y if y < value else value
    return x if x > n else n


def _matrix2euler(m):
    m11 = m[0, 0]
    m12 = m[0, 1]
    m13 = 0

    m21 = m[1, 0]
    m22 = m[1, 1]
    m23 = 0

    m31 = m[2, 0]
    m32 = m[2, 1]
    m33 = 0

    x = 0
    y = 0
    z = 0

    y = asin(-_clamp(m31, -1, 1))
    if fabs(m31) < 0.9999999:
        x = atan2(m32, m33)
        z = atan2(m21, m11)
    else:
        x = 0
        z = atan2(-m12, m22)

    return x, y, z


def _transform(links, i, theta):
    rz = np.array([
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta), 0],
        [0, 0, 1]
    ])

    px = 0
    if i != 0:
        px = links[i - 1]

    tx = np.array([
        [1, 0, px],
        [0, 1, 0],
        [0, 0, 1]
    ])
    return rz.dot(tx)


def forward(links, joints):
    """
    正解
    :param links: 连杆列表，每根连杆的长度
    :param joints: 关节角度
    :return:
    """
    if not isinstance(links, list) and not isinstance(links, tuple):
        raise Exception("link error")

    if not isinstance(joints, list) and not isinstance(joints, tuple):
        raise Exception("joint error")

    if len(links) != len(joints):
        raise Exception("joint and link num error")

    mtx = np.identity(3)

    for i in range(len(links) + 1):
        angle = 0
        if i != 0:
            angle = radians(joints[i - 1])
        mtx = mtx.dot(_transform(links, i, angle))

    euler = _matrix2euler(mtx)
    return mtx[0, 2], mtx[1, 2], degrees(euler[2])

if __name__ == '__main__':
    f = open('data2.csv','w')
    print(forward((2, 1.5), (30, 30)))
    for i in range(0,360):
        for j in range(0,360):
            x,y,theta = forward((2, 1.5), (i, j))
            f.write('{},{},{},{},{}\n'.format(i,j,x,y,theta))
    
    
    f.close()