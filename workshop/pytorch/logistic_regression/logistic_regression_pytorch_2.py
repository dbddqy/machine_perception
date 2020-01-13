import torch
import numpy as np
from matplotlib import pyplot as plt


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
    # transform into torch.tensor
    _x_train = torch.from_numpy(_x_train)
    _y_train = torch.from_numpy(_y_train)
    _x_test = torch.from_numpy(_x_test)
    _y_test = torch.from_numpy(_y_test)
    _x_train = _x_train.type(torch.float32)
    _y_train = _y_train.type(torch.float32)
    _x_test = _x_test.type(torch.float32)
    _y_test = _y_test.type(torch.float32)
    return _x_train, _y_train, _x_test, _y_test


def error_rate(_x, _y, _model):
    _error_count = 0
    for i in range(_x.shape[0]):
        _y_label = _y[i]
        _y_pred = _model.forward(_x[i])
        if (_y_pred[0] > 0.5) and (_y_label[0] == 0):
            _error_count += 1
        if (_y_pred[1] > 0.5) and(_y_label[1] == 0):
            _error_count += 1
    _error_rate = _error_count / _x.shape[0]
    return _error_rate


def plot_boundary(_model, _color):
    x1_plot = np.arange(-2., 10., .1)
    x2_plot = np.arange(-6., 6., .1)
    x1_plot, x2_plot = np.meshgrid(x1_plot, x2_plot)
    f = np.zeros(x1_plot.shape)
    for i in range(x1_plot.shape[0]):
        for j in range(x1_plot.shape[1]):
            # f[i][j] = model(np.array([[x1_plot[i][j]], [x2_plot[i][j]]]), _theta)[3][0][0] - 0.5
            f[i][j] = model.forward(torch.tensor([x1_plot[i][j], x2_plot[i][j]]))[0] - 0.5

    plt.contour(x1_plot, x2_plot, f, 0, colors=_color)


class Model(torch.nn.Module):
    def __init__(self):
        super(Model, self).__init__()
        self.linear = torch.nn.Linear(2, 2)

    def forward(self, x):
        # y_pred = torch.sigmoid(self.linear(x))
        y_pred = torch.softmax(self.linear(x), dim=0)
        return y_pred


    def li(self, x):
        return self.linear(x)



if __name__ == "__main__":
    # my model
    model = Model()

    # define criterion and optimizer
    criterion = torch.nn.BCELoss(reduction='mean')
    optimizer = torch.optim.SGD(model.parameters(), lr=1e-3)

    # generate data
    x_train, y_train, x_test, y_test = generate_data(100)


    for epoch in range(5000):
        y_pred = model(x_train)

        loss = criterion(y_pred, y_train)
        if (epoch % 100 == 0):
            print(f'Epoch: {epoch} | Loss: {loss.item()}')
            print("training error rate %f " % (error_rate(x_train, y_train, model)))
            print("testing error rate %f " % (error_rate(x_test, y_test, model)))

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    # visualize
    # x_boundary = [0., 10.]
    # y_boundary = [model.li(torch.tensor([0., 0.]))[0], model.li(torch.tensor([10., 10.]))[0]]
    #
    # plt.plot(x_boundary, y_boundary)

    plot_boundary(model, "green")
    plt.show()

    print("weight:  ", model.linear.weight)
    print("bias:  ", model.linear.bias)
    print("test result:  ", model.forward(torch.tensor([1., 3.])))




