import numpy as np
from matplotlib import pyplot as plt
from math import exp


def generate_data(n, p=0.8):
    # p: percentage of data for training
    x1 = np.random.multivariate_normal([4., 3.], [[4., 0.], [0., 1.]], n)
    plt.scatter(x1.T[0], x1.T[1], color="red")
    x2 = np.random.multivariate_normal([6., 0.], [[1.5, 0.5], [0.5, 1.5]], n)
    plt.scatter(x2.T[0], x2.T[1], color="blue")
    # combine data
    x = np.vstack((x1, x2))
    y = np.asarray([[1., 0.]] * n + [[0., 1.]] * n)
    # shuffle data
    shuffle_idx = np.arange(0, n*2)
    np.random.shuffle(shuffle_idx)
    x_shuffled = x[shuffle_idx]
    y_shuffled = y[shuffle_idx]
    # split data into training and testing
    _x_train = x_shuffled[0:int(n * p)*2]
    _y_train = y_shuffled[0:int(n * p)*2]
    _x_test = x_shuffled[int(n * p)*2:n*2]
    _y_test = y_shuffled[int(n * p)*2:n*2]
    return _x_train, _y_train, _x_test, _y_test


def model(_x, _theta):
    return 1. / (1. + exp(-_x[0] * _theta[0] - _x[1] * _theta[1] - _theta[2]))


def error_rate(_x, _y, _theta, _model):
    error_count = 0
    for i in range(len(_x)):
        if (_model(_x[i], _theta) > 0.5) and _y[i][0] == 0:
            error_count += 1
        if (_model(_x[i], _theta) < 0.5) and _y[i][0] == 1:
            error_count += 1
    return error_count / len(_x)


def gradient(_x, _y, _theta, _model):
    grad = np.array([0., 0., 0.])
    for i in range(len(_x)):
        grad[0] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1]) * _x[i][0]
        grad[1] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1]) * _x[i][1]
        grad[2] += (-(1. - model(_x[i], _theta)) * _y[i][0] + model(_x[i], _theta) * _y[i][1])
    return grad


def train(_x, _y, _theta, _model, lr=1e-5, steps=1000):
    global x_test, y_test
    for step in range(steps):
        _theta -= lr * gradient(_x, _y, _theta, _model)
        # print log
        if step % 100 == 0:
            print("step: %d, theta: %f %f %f" % (step, _theta[0], _theta[1], theta[2]))
            print("training error rate: %f, testing error rate: %f"
                  % (error_rate(_x, _y, _theta, _model), error_rate(x_test, y_test, _theta, _model)))
    return _theta


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    theta = np.array([1., 1., 0.])
    # training
    theta_new = train(x_train, y_train, theta, model, lr=1e-4, steps=1000)
    # plot boundary
    x_boundary = [0., 10.]
    y_boundary = [-theta_new[2]/theta_new[1], (-theta_new[2]-10*theta_new[0])/theta_new[1]]
    plt.plot(x_boundary, y_boundary)
    plt.show()

