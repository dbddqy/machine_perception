import numpy as np
from matplotlib import pyplot as plt
import math


def generate_data(n, p=0.8):
    # p: percentage of data for training
    x1 = np.random.multivariate_normal([4., 3.], [[4., 0.], [0., 1.]], n)
    plt.scatter(x1.T[0], x1.T[1], marker="^", color="red")
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


# sigmoid function
def sigmoid_func(z):
    return 1. / (1. + np.exp(-z))


# logistic regression hypothesis
def sigmoid(_x, _theta):
    return sigmoid_func(_theta[0] * _x[0] + _theta[1] * _x[1] + _theta[2])


# to calculate y (1 x 2) from x (1 x 2) and theta (1 x 3)
def model(_x, _theta):
    _y0 = sigmoid(_x, _theta)
    _y1 = 1 - sigmoid(_x, _theta)
    _y = [_y0, _y1]
    return _y


def cost(_x, _y, _theta, _model=model):
    _loss = 0.0
    for i in range(len(_x)):
        _y_label = _y[i]
        _y_predict = _model(_x[i], _theta)
        _loss += -(_y_label[0] * math.log(_y_predict[0]) + _y_label[1] * math.log(_y_predict[1]))
    return (1 / len(_x)) * _loss


def error_rate(_x, _y, _theta, _model=model):
    _error_count = 0
    for i in range(len(_x)):
        _y_label = _y[i]
        _y_predict = _model(_x[i], _theta)
        if (_y_predict[0] > 0.5) and (_y_label[0] == 0):
            _error_count += 1
        if (_y_predict[1] > 0.5) and(_y_label[1] == 0):
            _error_count += 1
    _error_rate = _error_count / len(_x)
    return _error_rate


def gradient(_x, _y, _theta):
    _g = np.array([0., 0., 0.])
    for i in range(len(_x)):
        _g[0] += (-(1. - sigmoid(_x[i], _theta)) * _y[i][0] + sigmoid(_x[i], _theta) * _y[i][1]) * _x[i][0]
        _g[1] += (-(1. - sigmoid(_x[i], _theta)) * _y[i][0] + sigmoid(_x[i], _theta) * _y[i][1]) * _x[i][1]
        _g[2] += (-(1. - sigmoid(_x[i], _theta)) * _y[i][0] + sigmoid(_x[i], _theta) * _y[i][1]) * 1
    return _g


def train(_x, _y, _theta, _lr=1e-5, _steps=1500):
    global x_test, y_test
    for step in range(_steps):
        _theta -= _lr * gradient(_x, _y, _theta)
        # print log
        if step % 100 == 0:
            print("step %d, theta [%f, %f, %f]" % (step, _theta[0], _theta[1], _theta[2]))
            print("cost", cost(_x, _y, _theta))
            print("training error rate %f, test error rate %f \n" % (error_rate(_x, _y, _theta), error_rate(x_test, y_test, _theta)))
    return _theta


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    theta_original = [2., -1., 0.]
    theta_new = train(x_train, y_train, theta_original)
    
    print("original cost", cost(x_train, y_train, theta_original, _model=model))
    print("new cost", cost(x_train, y_train, theta_new, _model=model))
    print("new theta", theta_new)

    x_boundary = [0., 10.]
    y_boundary = [-theta_new[2]/theta_new[1], (-theta_new[2]-10*theta_new[0])/theta_new[1]]
    plt.plot(x_boundary, y_boundary)
    plt.show()
