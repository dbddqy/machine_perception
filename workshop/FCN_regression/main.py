import numpy as np
from matplotlib import pyplot as plt
from math import sin


def generate_data(n, low=0.0, high=3.0):
    step = (high - low) / n
    x, y = [], []
    for i in range(n):
        x.append(low + step * i)
        y.append(sin(x[i] * x[i] + 1.) + np.random.normal(0.0, 0.1))
    return np.array(x), np.array(y)


def sigmoid(_x):
    return 1. / (1. + np.exp(-_x))


def model(_x, _theta):
    """
    model architecture:
    input layer: m0 = 1
        theta["w1"]: (50, 1), theta["b1"]: (50, 1)
    hidden layer: m1 = 50, activation: sigmoid
        theta["w2"]: (5, 50), theta["b2"]: (5, 1)
    hidden layer: m2 = 5, activation: sigmoid
        theta["w3"]: (1, 5), theta["b3"]: (1, 1)
    output layer: m3 = 1
    """
    x0 = np.array(_x).reshape([1, 1])
    x1 = sigmoid(np.dot(_theta["w1"], x0) + _theta["b1"])
    x2 = sigmoid(np.dot(_theta["w2"], x1) + _theta["b2"])
    x3 = np.dot(_theta["w3"], x2) + _theta["b3"]
    return x0, x1, x2, x3


def loss(_x, _y, _theta, _model):
    _loss = 0.0
    for i in range(len(_x)):
        _loss += (model(_x[i], _theta)[3][0][0] - _y[i]) ** 2
    return _loss / len(_x)


def gradient(_x, _y, _theta, _model):
    grad = {"w1": np.zeros((50, 1)), "b1": np.zeros((50, 1))
            , "w2": np.zeros((5, 50)), "b2": np.zeros((5, 1))
            , "w3": np.zeros((1, 5)), "b3": np.zeros((1, 1))}
    for i in range(len(_x)):
        x0, x1, x2, x3 = model(_x[i], _theta)
        # back propagation
        _loss_to_x3 = -2 * (np.array(_y[i]).reshape([1, 1]) - x3)
        grad["w3"] += np.dot(_loss_to_x3.T, x2.T)
        grad["b3"] += _loss_to_x3.T
        _x3_to_x2 = _theta["w3"]
        _x2_to_a2 = np.diag((x2 - x2 * x2).reshape([5, ]))
        _loss_to_a2 = _loss_to_x3.dot(_x3_to_x2).dot(_x2_to_a2)
        grad["w2"] += np.dot(_loss_to_a2.T, x1.T)
        grad["b2"] += _loss_to_a2.T
        _a2_to_x1 = _theta["w2"]
        _x1_to_a1 = np.diag((x1 - x1 * x1).reshape([50, ]))
        _loss_to_a1 = _loss_to_a2.dot(_a2_to_x1).dot(_x1_to_a1)
        grad["w1"] += np.dot(_loss_to_a1.T, x0.T)
        grad["b1"] += _loss_to_a1.T
    return grad


def train(_x, _y, _theta, _model, lr=1e-5, steps=1000):
    for step in range(steps):
        grad = gradient(_x, _y, _theta, _model)
        _theta["w1"] -= lr * grad["w1"]
        _theta["b1"] -= lr * grad["b1"]
        _theta["w2"] -= lr * grad["w2"]
        _theta["b2"] -= lr * grad["b2"]
        _theta["w3"] -= lr * grad["w3"]
        _theta["b3"] -= lr * grad["b3"]
        # print log
        if step % 100 == 0:
            print("step: %d, loss: %f" % (step, loss(_x, _y, _theta, _model)))
    return _theta


if __name__ == "__main__":
    x_train, y_train = generate_data(100)
    # plot x, y data
    plt.scatter(x_train, y_train, color="red")
    # parameter init according to model
    # weight matrix initialization with uniform[0, 1] fails
    theta = {"w1": np.random.uniform(-1., 1., [50, 1]), "b1": np.zeros([50, 1])
             , "w2": np.random.uniform(-1., 1., [5, 50]), "b2": np.zeros([5, 1])
             , "w3": np.random.uniform(-1., 1., [1, 5]), "b3": np.zeros([1, 1])}
    # # plot original curve
    # y_plot = []
    # for value in x_train:
    #     y_plot.append(model(value, theta)[3][0][0])
    # plt.plot(x_train, y_plot)
    # training
    theta_new = train(x_train, y_train, theta, model, lr=1e-3, steps=10000)
    # plot trained curve
    y_plot = []
    for value in x_train:
        y_plot.append(model(value, theta_new)[3][0][0])
    plt.plot(x_train, y_plot)
    plt.show()

