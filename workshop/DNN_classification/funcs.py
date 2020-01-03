import numpy as np


def relu(x):  # (x, 1)
    return np.maximum(0., x)  # (x, 1)


def d_relu(x):  # (x, 1)
    return np.diag(np.where(x > 0., 1., 0.).T[0])  # (x, x)


def sigmoid(x):  # (x, 1)
    return np.ones(x.shape) / (np.ones(x.shape) + np.exp(-x))


def d_sigmoid(x):
    return np.diag((x - x*x).T[0])  # (x, x)


def softmax(x):  # (x, 1)
    return np.exp(x) / np.sum(np.exp(x))  # (x, 1)


def d_softmax(x):  # (x, 1)
    return np.diag(x.T[0]) - x.dot(x.T)  # (x, x)


def l2(x, y):  # (x, 1) (x, 1)
    return np.sum((x - y) ** 2)  # scalar


def d_l2(x, y):  # (x, 1)
    return 2 * (x - y).T  # (1, x)


def cross_entropy(x, y):  # (x, 1) (x, 1)
    return -np.sum(y * np.log(x))  # scalar


def d_cross_entropy(x, y):  # (x, 1)
    return -(y / x).T  # (1, x)


# a = [2]
# print(a[-1])
# a = np.array([[0.2], [0.8]])
# b = np.array([[1], [0]])
#
# print(d_cross_entropy(a, b))

# c = [a, b]
#
# d = c[:]
# d[1] += a
# print(c)
# print(d)
# print(l2(a, b))
