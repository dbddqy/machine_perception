import numpy as np
from cvxopt import solvers, matrix
from matplotlib import pyplot as plt
import random
import struct
import gzip
import pickle


def read_idx(filename):
    with gzip.open(filename) as f:
        zero, data_type, dims = struct.unpack('>HBB', f.read(4))
        shape = tuple(struct.unpack('>I', f.read(4))[0] for d in range(dims))
        return np.fromstring(f.read(), dtype=np.uint8).reshape(shape)


def count_label(labels):
    counts = [[], [], [], [], [], [], [], [], [], []]
    for index in range(len(labels)):
        counts[labels[index]].append(index)
    return counts


def kernel_RBF(x1, x2, _gamma):
    return np.exp(-_gamma * np.sum((x1 - x2) * (x1 - x2)))


def kernel_polynomial(x1, x2, _gamma):
    return (1 + np.sum(x1*x2)) ** 2


def get_P_Mat(x, y, kernel, _gamma):
    n = y.shape[0]
    _P = np.zeros([n, n], dtype=np.float64)
    for i in range(n):
        for j in range(n):
            _P[i][j] = y[i][0] * y[j][0] * kernel(x[i], x[j], _gamma)
    return _P


def getSamples(labels):
    global x_train_reduced, y_train
    # choose 1:1 samples for certain label : the rest
    x_choose = x_train_reduced[labels]
    indices = []
    while len(indices) < len(labels):
        next_index = random.randint(0, y_train.shape[0]-1)
        if next_index not in labels:
            indices.append(next_index)
    x_rest = x_train_reduced[indices]
    x_mixed = np.vstack((x_choose, x_rest))
    y_mixed = np.asarray([[1.]] * len(labels) + [[-1.]] * len(labels))
    shuffle_idx = np.arange(0, len(labels) * 2)
    np.random.shuffle(shuffle_idx)
    x_shuffled = x_mixed[shuffle_idx]
    y_shuffled = y_mixed[shuffle_idx]
    return x_shuffled, y_shuffled


def trainSVM(x, y, _kernel, _gamma, _C):
    n = y.shape[0]
    P_Mat = get_P_Mat(x, y, kernel=_kernel, _gamma=_gamma)
    # optimizaton
    P = matrix(P_Mat)
    q = matrix(-np.ones([n, 1]))
    # 2*n inequality constraints
    G = matrix(np.vstack([-np.eye(n), np.eye(n)]))
    h = matrix(np.vstack([np.zeros([n, 1]), _C * np.ones([n, 1])]))
    # 1 equality constraint
    A = matrix(y.T)
    b = matrix(0.)
    sol = solvers.qp(P, q, G, h, A, b)
    alpha = np.array(sol['x'])
    # get SVs and SOs
    SV, SO = [], []
    for i in range(n):
        if alpha[i][0] < 0.001:
            continue
        if abs(alpha[i][0] - _C) > 0.001:
            SV.append(i)
        SO.append(i)
    # calculate b
    w0 = 0.
    for i in SV:
        w0 += y[i][0]
        for j in SO:
            w0 -= alpha[j][0] * y[j][0] * P_Mat[j][i]
    w0 /= len(SV)
    return alpha[SO], x[SO], y[SO], w0


def predict(x, label, kernel, _gamma):
    global alphas, SOs, ws
    global x_train_reduced, y_train
    pred = ws[label]
    for index in range(alphas[label].shape[0]):
        pred += (alphas[label][index][0] * y_train[SOs[label][index]] * kernel(x_train_reduced[SOs[label][index]], x, _gamma))
    return pred


class TenClassSVM:
    def __init__(self, _alphas, _xs, _ys, _ws, _gamma, _kernel, _C):
        self.gamma = _gamma
        self.kernel = _kernel
        self.C = _C
        self.alphas = _alphas
        self.xs = _xs
        self.ys = _ys
        self.ws = _ws

    def predict(self, x, _label):
        alpha_use = self.alphas[_label]
        x_use = self.xs[_label]
        y_use = self.ys[_label]
        pred = self.ws[_label]
        for index in range(alpha_use.shape[0]):
            pred += (alpha_use[index][0] * y_use[index][0] * self.kernel(x_use[index], x, self.gamma))
        return pred


if __name__ == "__main__":
    x_total = read_idx("./MNIST/t10k-images-idx3-ubyte.gz")
    y_total = read_idx("./MNIST/t10k-labels-idx1-ubyte.gz")
    N = 2000
    x_train, y_train = x_total[:N], y_total[:N]
    x_test, y_test = x_total[N:N+100], y_total[N:N+100]

    # PCA
    x_train_flattened = np.array(x_train.reshape([N, 28*28, 1]), dtype=np.float32)
    R = np.einsum("ijk, ilk -> ijl", x_train_flattened, x_train_flattened).mean(axis=0)
    e_vals, e_vecs = np.linalg.eig(R)
    W = e_vecs[:, :50]
    x_train_reduced = W.T.dot(x_train_flattened.reshape([N, 28*28]).T).T

    # x1 = np.array(x_test.reshape([100, 28*28, 1]), dtype=np.float32)[4]
    # plt.imshow(x1.reshape([28, 28]), cmap="Greys")
    # plt.show()
    #
    # x1_after = W.dot(W.T).dot(x1).reshape([28, 28])
    # plt.imshow(x1_after, cmap="Greys")
    # plt.show()

    # SVM training

    gamma = 9.0e-7
    kernel = kernel_RBF
    C = 1

    alphas, xs, ys, ws = [], [], [], []
    label_counts = count_label(y_train)
    for label in range(10):
        x, y = getSamples(label_counts[label])
        alpha_t, x_t, y_t, w_t = trainSVM(x, y, kernel, gamma, C)
        alphas.append(alpha_t)
        xs.append(x_t)
        ys.append(y_t)
        ws.append(w_t)

    # save model

    model = TenClassSVM(alphas, xs, ys, ws, gamma, kernel, C)
    f = open('model_1.pickle', 'wb')
    pickle.dump(model, f)
    f.close()

    # load model

    f = open('model_1.pickle', 'rb')
    model = pickle.load(f)
    f.close()

    print(model.alphas[0].shape)
    print(model.ws)

    # SVM prediction

    error_count = 0
    for i in range(N):
        predictions = []
        for label in range(10):
            predictions.append(model.predict(x_train_reduced[i], label))
        if y_train[i] != predictions.index(max(predictions)):
            error_count += 1
        # print(predictions)
        # print(y_train[i])
    error_rate = float(error_count) / x_train_reduced.shape[0]
    print("training error rate: %f" % error_rate)

    # testing set

    x_test_reduced = W.T.dot(np.array(x_test.reshape([100, 28*28]).T, dtype=np.float32)).T
    error_count = 0
    for i in range(x_test_reduced.shape[0]):
        predictions = []
        for label in range(10):
            predictions.append(model.predict(x_test_reduced[i], label))
        if y_test[i] != predictions.index(max(predictions)):
            error_count += 1
        # print(predictions)
        # print(y_test[i])
    error_rate = float(error_count) / x_test_reduced.shape[0]
    print("test error rate: %f" % error_rate)
