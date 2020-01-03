from NeuralNetwork import *
from matplotlib import pyplot as plt


def generate_data(n, p=0.8):
    # p: percentage of data for training
    x11 = np.random.multivariate_normal([4., 3.], [[4., 0.], [0., 1.]], int(0.5*n))
    x12 = np.random.multivariate_normal([2., -2.], [[1., 0.], [0., 2.]], int(0.25*n))
    x13 = np.random.multivariate_normal([7., -4.], [[1., 0.], [0., 1.]], int(0.25*n))
    x1 = np.vstack((x11, x12, x13))
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


def error_rate(_x, _y, _dnn):
    error_count = 0
    for i in range(len(_x)):
        if (_dnn.model(_x[i:i+1].T)[-1][0][0] > 0.5) and _y[i][0] == 0:
            error_count += 1
        if (_dnn.model(_x[i:i+1].T)[-1][0][0] < 0.5) and _y[i][0] == 1:
            error_count += 1
    return error_count / len(_x)


def train(_x, _y, _dnn, lr=1e-5, steps=1000):
    global x_test, y_test
    # using adagrad
    grad_w_sum_square, grad_b_sum_square = [], []
    for i in range(len(_dnn.Ws)):
        grad_w_sum_square.append(np.zeros(_dnn.Ws[i].shape))
        grad_b_sum_square.append(np.zeros(_dnn.Bs[i].shape))
    for step in range(steps):
        grad_w, grad_b = _dnn.gradient(_x, _y, d_loss_f=d_cross_entropy)
        for i in range(len(_dnn.Ws)):
            grad_w_sum_square[i] += grad_w[i] ** 2
            grad_b_sum_square[i] += grad_b[i] ** 2
            _dnn.Ws[i] -= grad_w[i] * lr / (np.sqrt(grad_w_sum_square[i])+1e-9)
            _dnn.Bs[i] -= grad_b[i] * lr / (np.sqrt(grad_b_sum_square[i])+1e-9)
        # print log
        if step % 100 == 0:
            print("step: %d, training error rate: %f, testing error rate: %f"
                  % (step, error_rate(_x, _y, _dnn), error_rate(x_test, y_test, _dnn)))


def plot_boundary(_dnn, _color):
    x_plot = np.arange(-2., 10., .1)
    y_plot = np.arange(-6., 6., .1)
    x_plot, y_plot = np.meshgrid(x_plot, y_plot)
    f = np.zeros(x_plot.shape)
    for i in range(x_plot.shape[0]):
        for j in range(x_plot.shape[1]):
            f[i][j] = _dnn.model(np.array([[x_plot[i][j]], [y_plot[i][j]]]))[-1][0][0] - 0.5
    plt.contour(x_plot, y_plot, f, 0, colors=_color)


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    dnn = NeuralNetwork(2)
    for _ in range(2):
        dnn.add_dense_layer(100, activation=sigmoid, d_activation=d_sigmoid)
    dnn.add_dense_layer(2, activation=softmax, d_activation=d_softmax)
    # plot boundary before training
    plot_boundary(dnn, "orange")
    # training
    # print(dnn.model(x_train[6:7].T)[-1])
    train(x_train, y_train, dnn, lr=5e-4, steps=1000)
    # plot boundary after training
    plot_boundary(dnn, "lightblue")
    plt.show()
