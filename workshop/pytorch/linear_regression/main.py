import numpy as np
from matplotlib import pyplot as plt


# generate data for training
def generate_data(n, k, b, low=0.0, high=10.0):
    step = (high - low) / n
    x, y = [], []
    for i in range(n):
        x.append(low + step * i)
        y.append(x[i] * k + b + np.random.normal(0.0, 0.05 * (high - low)))
    return np.array(x), np.array(y)


# set a function as model
def f_x(_x, _theta):
    _y = _x * _theta[0] + _theta[1]
    return _y


# calculate loss between training data and result from model
def loss(_x_train, _y_train, _theta, _f_x=f_x):
    _loss = 0.0
    for i in range(len(_x_train)):
        _loss += (_f_x(_x_train[i], _theta) - _y_train[i]) ** 2
    return _loss / len(_x_train)


# calculate gradient (d(Loss) / d(k), d(Loss) / d(b))
def gradient(_x_train, _y_train, _theta, _f_x=f_x):
    _k = _theta[0]
    _b = _theta[1]
    _delta_k = 0.0
    _delta_b = 0.0
    for i in range(len(_x_train)):
        _delta_k += 2 * (_f_x(_x_train[i], _theta) - _y_train[i]) * _x_train[i]
        _delta_b += 2 * (_f_x(_x_train[i], _theta) - _y_train[i]) * 1
    _gradient = np.array([_delta_k, _delta_b])
    return _gradient


def train(_x_train, _y_train, _theta, _f_x=f_x, _lr=1e-6, _step=1000):
    for i in range(_step):
        _theta = _theta - _lr * gradient(_x_train, _y_train, _theta, _f_x)
        print(loss(_x_train, _y_train, _theta, _f_x))
    return _theta


if __name__ == "__main__":
    x_train, y_train = generate_data(100, 2.0, 0.5)
    theta0 = [5.0, 1.0]
    theta = train(x_train, y_train, theta0)

    print(theta)

    #plot training data as red points
    plt.title("Linear Regression")
    plt.xlabel("Axis X")
    plt.ylabel("Axis Y")
    plt.plot(x_train, y_train, "ob", color="red")
    plt.plot(x_train, f_x(x_train, theta), color="blue")
    plt.show()