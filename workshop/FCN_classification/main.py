import numpy as np
from matplotlib import pyplot as plt


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


def sigmoid(_x):
    return 1. / (1. + np.exp(-_x))


def softmax(_x):
    return np.exp(_x) / np.sum(np.exp(_x))


def model(_x, _theta):
    """
    model architecture:
    input layer: m0 = 2
        theta["w1"]: (50, 2), theta["b1"]: (50, 1)
    hidden layer: m1 = 50, activation: sigmoid
        theta["w2"]: (5, 50), theta["b2"]: (5, 1)
    hidden layer: m2 = 5, activation: sigmoid
        theta["w3"]: (2, 5), theta["b3"]: (2, 1)
    output layer: m3 = 2, activation: soft-max
    """
    x0 = np.array(_x).reshape([2, 1])
    x1 = sigmoid(np.dot(_theta["w1"], x0) + _theta["b1"])
    x2 = sigmoid(np.dot(_theta["w2"], x1) + _theta["b2"])
    x3 = softmax(np.dot(_theta["w3"], x2) + _theta["b3"])
    return x0, x1, x2, x3


def error_rate(_x, _y, _theta, _model):
    error_count = 0
    for i in range(len(_x)):
        if (_model(_x[i], _theta)[3][0][0] > 0.5) and _y[i][0] == 0:
            error_count += 1
        if (_model(_x[i], _theta)[3][0][0] < 0.5) and _y[i][0] == 1:
            error_count += 1
    return error_count / len(_x)


def gradient(_x, _y, _theta, _model):
    grad = {"w1": np.zeros([50, 2]), "b1": np.zeros([50, 1])
            , "w2": np.zeros([5, 50]), "b2": np.zeros([5, 1])
            , "w3": np.zeros([2, 5]), "b3": np.zeros([2, 1])}
    for i in range(len(_x)):
        x0, x1, x2, x3 = model(_x[i].reshape([2, 1]), _theta)
        # back propagation
        _loss_to_x3 = (-_y[i].reshape([2, 1]) / x3).T
        _x3_to_a3 = np.diag(x3.reshape([2, ])) - x3.dot(x3.T)
        _loss_to_a3 = _loss_to_x3.dot(_x3_to_a3)
        grad["w3"] += np.dot(_loss_to_a3.T, x2.T)
        grad["b3"] += _loss_to_a3.T
        _a3_to_x2 = _theta["w3"]
        _x2_to_a2 = np.diag((x2 - x2 * x2).reshape([5, ]))
        _loss_to_a2 = _loss_to_a3.dot(_a3_to_x2).dot(_x2_to_a2)
        grad["w2"] += np.dot(_loss_to_a2.T, x1.T)
        grad["b2"] += _loss_to_a2.T
        _a2_to_x1 = _theta["w2"]
        _x1_to_a1 = np.diag((x1 - x1 * x1).reshape([50, ]))
        _loss_to_a1 = _loss_to_a2.dot(_a2_to_x1).dot(_x1_to_a1)
        grad["w1"] += np.dot(_loss_to_a1.T, x0.T)
        grad["b1"] += _loss_to_a1.T
    return grad


def train(_x, _y, _theta, _model, lr=1e-5, steps=1000):
    global x_test, y_test
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
            print("step: %d, training error rate: %f, testing error rate: %f"
                  % (step, error_rate(_x, _y, _theta, _model), error_rate(x_test, y_test, _theta, _model)))
    return _theta


def plot_boundary(_theta, _color):
    x_plot = np.arange(-2., 10., .1)
    y_plot = np.arange(-2., 10., .1)
    x_plot, y_plot = np.meshgrid(x_plot, y_plot)
    f = np.zeros(x_plot.shape)
    for i in range(x_plot.shape[0]):
        for j in range(x_plot.shape[1]):
            f[i][j] = model(np.array([[x_plot[i][j]], [y_plot[i][j]]]), _theta)[3][0][0] - 0.5
    plt.contour(x_plot, y_plot, f, 0, colors=_color)


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    # parameter init according to model
    # weight matrix initialization with uniform[0, 1] fails
    theta = {"w1": np.random.uniform(-1., 1., [50, 2]), "b1": np.zeros([50, 1])
             , "w2": np.random.uniform(-1., 1., [5, 50]), "b2": np.zeros([5, 1])
             , "w3": np.random.uniform(-1., 1., [2, 5]), "b3": np.zeros([2, 1])}
    # plot boundary before training
    plot_boundary(theta, "orange")
    # training
    theta_new = train(x_train, y_train, theta, model, lr=1e-4, steps=1000)
    # plot boundary after training
    plot_boundary(theta_new, "lightblue")
    plt.show()
