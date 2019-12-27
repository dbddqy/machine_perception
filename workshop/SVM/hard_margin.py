import numpy as np
from cvxopt import solvers, matrix
from matplotlib import pyplot as plt


def generate_data(n, p=1.0):
    # p: percentage of data for training

    # x11 = np.random.multivariate_normal([4., 3.], [[4., 0.], [0., 1.]], int(0.5*n))
    # x12 = np.random.multivariate_normal([2., -2.], [[1., 0.], [0., 2.]], int(0.25*n))
    # x13 = np.random.multivariate_normal([7., -4.], [[1., 0.], [0., 1.]], int(0.25*n))
    # x1 = np.vstack((x11, x12, x13))
    x1 = np.random.multivariate_normal([4., 3.], [[.4, 0.], [0., .1]], n)  # simple case

    plt.scatter(x1.T[0], x1.T[1], color="red")
    x2 = np.random.multivariate_normal([6., 0.], [[.15, .05], [.05, .15]], n)
    plt.scatter(x2.T[0], x2.T[1], color="blue")
    # combine data
    x = np.vstack((x1, x2))
    y = np.asarray([[1.]] * n + [[-1.]] * n)
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


if __name__ == "__main__":
    n = 100
    x_train, y_train, x_test, y_test = generate_data(n)
    X = (x_train*y_train).T

    P = matrix(X.T.dot(X))
    q = matrix(-np.ones([n*2, 1]))
    G = matrix(-np.eye(n*2))
    h = matrix(np.zeros([n*2, 1]))

    A = matrix(y_train.T)
    b = matrix(0.)

    sol = solvers.qp(P, q, G, h, A, b)  # 调用优化函数solvers.qp求解
    alpha = np.array(sol['x'])
    # print(alpha)
    w = X.dot(alpha)
    # print(w)

    # calculate b
    count = 0
    b = 0.
    for i in range(n*2):
        if alpha[i][0] < 0.001:
            continue
        b += (y_train[i][0] - w.T.dot(x_train[i].T))
        count += 1
    b /= count

    # print(b)
    plt.plot([0., 6.], [-b/w[1][0], (-b-6.*w[0][0])/w[1][0]])
    plt.show()
