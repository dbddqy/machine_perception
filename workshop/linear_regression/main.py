import numpy as np
from matplotlib import pyplot as plt


def generate_data(n, k, b, low=0.0, high=10.0):
    step = (high - low) / n
    x, y = [], []
    for i in range(n):
        x.append(low + step * i)
        y.append(x[i] * k + b + np.random.normal(0.0, 0.05 * (high - low)))
    return np.array(x), np.array(y)


def model(_x, _theta):
    return _x * _theta[0] + _theta[1]


def loss(_x, _y, _theta, _model):
    _loss = 0.0
    for i in range(len(_x)):
        _loss += (model(_x[i], _theta) - _y[i]) ** 2
    return _loss / len(_x)


def gradient(_x, _y, _theta, _model):
    g = np.array([0.0, 0.0])
    for i in range(len(_x)):
        g[0] += (model(_x[i], _theta) - _y[i]) * _x[i]
        g[1] += (model(_x[i], _theta) - _y[i])
    return g


def train(_x, _y, _theta, _model, lr=1e-5, steps=1000):
    for step in range(steps):
        _theta -= lr * gradient(_x, _y, _theta, _model)
        # print log
        if step % 100 == 0:
            print("step: %d, theta: %f %f, loss: %f" % (step, _theta[0], _theta[1], loss(_x, _y, _theta, _model)))
    return _theta


if __name__ == "__main__":
    x_train, y_train = generate_data(100, 2.0, 0.5)
    theta = np.array([1.0, 0.0])
    # plot x, y data
    plt.scatter(x_train, y_train, color="red")
    # training
    theta_new = train(x_train, y_train, theta, model, steps=1000)
    # plot line
    y_new = model(x_train, theta_new)
    plt.plot(x_train, y_new)
    plt.show()

