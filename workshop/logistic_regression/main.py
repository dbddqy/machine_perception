import numpy as np
from matplotlib import pyplot as plt
from math import exp


def generate_data(n, p=0.8):
    # p: percentage of data for training
    mean = [4., 3.]
    cov = [[4., 0.], [0., 1.]]
    x1 = np.random.multivariate_normal(mean, cov, n)
    plt.scatter(x1.T[0], x1.T[1], color="red")
    mean = [6., 0.]
    cov = [[1.5, 0.5], [0.5, 1.5]]
    x2 = np.random.multivariate_normal(mean, cov, n)
    plt.scatter(x2.T[0], x2.T[1], color="blue")
    # combine data
    x = np.vstack((x1, x2))
    y = np.asarray([[1., 0.]] * n + [[0., 1.]] * n)
    # shuffle data
    shuffle_idx = np.arange(0, n)
    np.random.shuffle(shuffle_idx)
    x_shuffled = x[shuffle_idx]
    y_shuffled = y[shuffle_idx]
    # split data into training and testing
    _x_train = x_shuffled[0:int(n * p)]
    _y_train = y_shuffled[0:int(n * p)]
    _x_test = x_shuffled[int(n * p):n]
    _y_test = y_shuffled[int(n * p):n]
    return _x_train, _y_train, _x_test, _y_test


def model(_x, _theta):
    return 1. / (1. + exp(-_x[0] * _theta[0] - _x[1] * _theta[1] - _theta[2]))


# def loss(_x, _y, _theta, _model):
#     _loss = 0.0
#     for i in range(len(_x)):
#         _loss += (model(_x[i], _theta) - _y[i]) ** 2
#     return _loss / len(_x)


def gradient(_x, _y, _theta, _model):
    g = np.array([0., 0., 0.])
    for i in range(len(_x)):
        g[0] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1]) * _x[i][0]
        g[1] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1]) * _x[i][1]
        g[2] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1])
    return g


def train(_x, _y, _theta, _model, lr=1e-3, steps=1000):
    for step in range(steps):
        _theta -= lr * gradient(_x, _y, _theta, _model)
        # print log
        if step % 100 == 0:
            print("step: %d, theta: %f %f %f, loss: %f" % (step, _theta[0], _theta[1], theta[2], 0.))
    return _theta


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    theta = np.array([1., 1., 0.])
    # training
    theta_new = train(x_train, y_train, theta, model, steps=1000)
    # plot boundary
    x_boundary = [0., 10.]
    y_boundary = [-theta_new[2]/theta_new[1], (-theta_new[2]-10*theta_new[0])/theta_new[1]]
    plt.plot(x_boundary, y_boundary)
    plt.show()


