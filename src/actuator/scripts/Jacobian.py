import numpy as np


def Jacobian(q):
    # 检查输入的关节角度是否合法
    if q.shape != (7,):
        print("关节角度输入错误！")
        return -1

    alfa = [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2]
    a = [0, 0, 0, 0, 0, 0, 0]
    d = [0.34, 0, 0.4, 0, 0.4, 0, 0.126]
    d[6] += 0.061

    T = np.zeros((4, 4, 7))
    T[:, :, 0] = getDHMatrix(alfa[0], q[0], d[0], a[0])

    for i in range(1, 7):
        T[:, :, i] = np.dot(T[:, :, i - 1], getDHMatrix(alfa[i], q[i], d[i], a[i]))
        T[:, :, i] = normalizeRotation(T[:, :, i])
        # print(T[:, :, i])

    J = np.zeros((6, 7))
    pef = T[:3, 3, 6]
    for i in range(6):
        k = T[:3, 2, i]
        pij = pef - T[:3, 3, i]
        J[:3, i] = np.cross(k, pij)
        J[3:, i] = k

    return J


def getDHMatrix(alfa, theta, d, a):
    T = np.zeros((4, 4))

    cosalfa = np.cos(alfa)
    sinalfa = np.sin(alfa)
    costheta = np.cos(theta)
    sintheta = np.sin(theta)

    T[0, 0] = costheta
    T[1, 0] = sintheta * cosalfa
    T[2, 0] = sintheta * sinalfa

    T[0, 1] = -sintheta
    T[1, 1] = costheta * cosalfa
    T[2, 1] = costheta * sinalfa

    T[1, 2] = -sinalfa
    T[2, 2] = cosalfa

    T[0, 3] = a
    T[1, 3] = -d * sinalfa
    T[2, 3] = d * cosalfa
    T[3, 3] = 1

    return T


def normalizeRotation(T):
    R = T[:3, :3]
    # normalize the columns of a rotation matrix
    for i in range(3):
        R[:, i] /= np.linalg.norm(R[:, i])

    T[:3, :3] = R
    return T
