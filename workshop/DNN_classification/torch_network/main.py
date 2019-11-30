from torch_network.TorchNN import *
from matplotlib import pyplot as plt
import numpy as np
import torch


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
    # make torch tensor
    _x_train = torch.tensor(_x_train.T, dtype=torch.float32)
    _y_train = torch.tensor(_y_train.T, dtype=torch.float32)
    _x_test = torch.tensor(_x_test.T, dtype=torch.float32)
    _y_test = torch.tensor(_y_test.T, dtype=torch.float32)
    return _x_train, _y_train, _x_test, _y_test


def error_rate(_x, _y, _dnn):
    y = _dnn.model(_x)
    error_count = 0
    for i in range(y.shape[1]):
        if (y[0, i].data.numpy() > 0.5) and _y[0, i].data.numpy() == 0:
            error_count += 1
        if (y[0, i].data.numpy() < 0.5) and _y[0, i].data.numpy() == 1:
            error_count += 1
    return error_count / _y.shape[1]


def plot_boundary(_dnn, _color):
    x_plot = np.arange(-2., 10., .2)
    y_plot = np.arange(-6., 6., .2)
    x_plot, y_plot = np.meshgrid(x_plot, y_plot)
    f = np.zeros(x_plot.shape)
    for i in range(x_plot.shape[0]):
        for j in range(x_plot.shape[1]):
            f[i][j] = _dnn.model(torch.tensor([[x_plot[i][j]], [y_plot[i][j]]], dtype=torch.float32))[0][0] - 0.5
    plt.contour(x_plot, y_plot, f, 0, colors=_color)


x_train, y_train, x_test, y_test = generate_data(100)

nn = TorchNN(2)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(5)
nn.add_dense_layer(2, activation=softmax)

# print(x_train[:, :5])
# print(nn.model(x_train[:, :5]))

# loss = -torch.trace(torch.log(nn.model(x_train)).T.mm(y_train))
# loss.backward()
# nn.Ws[0] = torch.tensor([1])
# loss.backward()
# nn.Ws[0].data.zero_()
# print(nn.Ws[0])
# print(nn.Ws[0].grad)
# nn.Ws[0].data.sub_(nn.Ws[0].grad.data*0.01)
# print(nn.Ws[0])
# loss = -torch.trace(torch.log(nn.model(x_train)).T.mm(y_train))
# loss.backward()
# print("______________")
# print(nn.Ws[0].grad)

plot_boundary(nn, "orange")

lr = 1e-5

for step in range(1500):
    loss = -torch.trace(torch.log(nn.model(x_train)).T.mm(y_train)) / x_train.shape[1]
    loss.backward()
    for tensor in nn.Ws:
        tensor.data.sub_(tensor.grad.data * lr)
        tensor.grad.data.zero_()
    for tensor in nn.Bs:
        tensor.data.sub_(tensor.grad.data * lr)
        tensor.grad.data.zero_()
    if step % 100 == 0:
        print("step: %d, training error rate: %f, testing error rate: %f"
              % (step, error_rate(x_train, y_train, nn), error_rate(x_test, y_test, nn)))
        print("loss : %f" % loss.data.numpy())

plot_boundary(nn, "lightblue")
plt.show()
