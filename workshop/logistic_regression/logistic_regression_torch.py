import torch
import numpy as np
from matplotlib import pyplot as plt


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
    return torch.tensor(1.) / (1. + torch.exp(-_x[0] * _theta[0] - _x[1] * _theta[1] - _theta[2]))


def loss(_y, _y_pred):
    return -_y[0] * torch.log(_y_pred) - _y[1] * torch.log(1.-_y_pred)


def error_rate(_x, _y, _theta, _model):
    error_count = 0
    for i in range(len(_x)):
        if (_model(_x[i], _theta).data > 0.5) and _y[i][0] == 0:
            error_count += 1
        if (_model(_x[i], _theta).data < 0.5) and _y[i][0] == 1:
            error_count += 1
    return error_count / len(_x)


def train(_x, _y, _theta, _model, lr=1e-5, steps=1000):
    global x_test, y_test
    for step in range(steps):
        cost = torch.tensor(0.)
        for index in range(_x.shape[0]):
            cost += loss(_y[index], model(_x[index], _theta))
        cost /= _x.shape[0]
        cost.backward()
        _theta.detach().sub_(lr * _theta.grad.data)
        _theta.grad.detach().zero_()
        # print log
        if step % 100 == 0:
            print("step: %d, theta: %f %f %f" % (step, _theta[0], _theta[1], _theta[2]))
            print("training error rate: %f, testing error rate: %f"
                  % (error_rate(_x, _y, _theta, _model), error_rate(x_test, y_test, _theta, _model)))
    return _theta


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    theta = torch.tensor([1., 1., 0.], requires_grad=True)
    # training
    theta_new = train(x_train, y_train, theta, model, lr=1e-3, steps=1000)
    # plot boundary
    x_boundary = [0., 10.]
    y_boundary = [-theta_new[2]/theta_new[1], (-theta_new[2]-10*theta_new[0])/theta_new[1]]
    plt.plot(x_boundary, y_boundary)
    plt.show()
