import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
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
    y = np.asarray([0] * n + [1] * n)
    # shuffle data
    shuffle_idx = np.arange(0, n*2)
    np.random.shuffle(shuffle_idx)
    x_shuffled = x[shuffle_idx]
    y_shuffled = y[shuffle_idx]
    # split data into training and testing
    _x_train = torch.tensor(x_shuffled[0:int(n * p)*2], dtype=torch.float32)
    _y_train = torch.tensor(y_shuffled[0:int(n * p)*2], dtype=torch.int64)
    _x_test = torch.tensor(x_shuffled[int(n * p)*2:n*2], dtype=torch.float32)
    _y_test = torch.tensor(y_shuffled[int(n * p)*2:n*2], dtype=torch.int64)
    return _x_train, _y_train, _x_test, _y_test


def error_rate(_x, _y, _net):
    error_count = 0
    for i in range(len(_x)):
        if (_net(_x[i])[0] > 0.5) and _y[i] == 1:
            error_count += 1
        if (_net(_x[i])[0] < 0.5) and _y[i] == 0:
            error_count += 1
    return error_count / len(_x)


def plot_boundary(_net, _color):
    x_plot = np.arange(-2., 10., .1)
    y_plot = np.arange(-6., 6., .1)
    x_plot, y_plot = np.meshgrid(x_plot, y_plot)
    f = np.zeros(x_plot.shape)
    for i in range(x_plot.shape[0]):
        for j in range(x_plot.shape[1]):
            f[i][j] = _net(torch.tensor([x_plot[i][j], y_plot[i][j]]))[0] - 0.5
    plt.contour(x_plot, y_plot, f, 0, colors=_color)


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(2, 5)
        self.fc2 = nn.Linear(5, 5)
        self.fc3 = nn.Linear(5, 5)
        self.fc4 = nn.Linear(5, 5)
        self.fc_out = nn.Linear(5, 2)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = self.fc_out(x)
        return F.softmax(x, dim=-1)


if __name__ == "__main__":
    x_train, y_train, x_test, y_test = generate_data(100)
    net = Net()

    # plot boundary before training
    plot_boundary(net, "orange")

    loss_function = nn.CrossEntropyLoss()
    optimizer = optim.SGD(net.parameters(), lr=1e-2, momentum=0.0)

    for step in range(10000):
        a = net(x_train)
        loss = loss_function(a, y_train)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        if step % 100 == 0:
            print("step: %d, training error rate: %f, testing error rate: %f"
                  % (step, error_rate(x_train, y_train, net), error_rate(x_test, y_test, net)))

    # plot boundary after training
    plot_boundary(net, "lightblue")

    plt.show()
