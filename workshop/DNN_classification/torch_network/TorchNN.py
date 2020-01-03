import torch
# import torch.nn as nn
import torch.nn.functional as F
# import torch.optim as optim
import numpy as np
# from matplotlib import pyplot as plt


class TorchNN:
    def __init__(self, dim_in):
        self.Ws = []
        self.Bs = []
        self.activations = []
        # self.d_activations = []
        self.dims = [dim_in]

    def add_dense_layer(self, n, activation=F.relu):
        # stdv = 1. / (self.dims[-1] ** 0.5)
        # w = torch.empty(n, self.dims[-1]).uniform_(-stdv, stdv)
        # w.requires_grad = True
        w = torch.normal(0., 2./self.dims[-1], [n, self.dims[-1]], requires_grad=True)
        self.Ws.append(w)
        self.Bs.append(torch.zeros(n, 1, requires_grad=True))
        self.activations.append(activation)
        self.dims.append(n)

    def model(self, x0):
        # x_temp = x0
        # x = [x_temp]
        x = x0
        for i in range(len(self.Ws)):
            x = self.activations[i](self.Ws[i].mm(x) + self.Bs[i].repeat(1, x0.shape[1]))
            # x.append(x)
        return x


def softmax(x):  # (x, 1)
    return torch.exp(x) / torch.sum(torch.exp(x), 0)  # (x, 1)


# a = torch.arange(1, 5).reshape(2, 2)[:, 0:1]
# b = a.repeat(1, 3)
# print(a)
# print(b)
# print(b.shape)

# a = torch.empty(2, 3).uniform_(-1., 1.)
# a.requires_grad = True
# b = torch.sum(a*2)
# b.backward()
# print(a.grad)
