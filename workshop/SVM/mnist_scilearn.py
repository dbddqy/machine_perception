import numpy as np
from sklearn import svm
from matplotlib import pyplot as plt
import struct
import gzip
import pickle


def read_idx(filename):
    with gzip.open(filename) as f:
        zero, data_type, dims = struct.unpack('>HBB', f.read(4))
        shape = tuple(struct.unpack('>I', f.read(4))[0] for d in range(dims))
        return np.fromstring(f.read(), dtype=np.uint8).reshape(shape)


if __name__ == "__main__":
    x_total = read_idx("./MNIST/t10k-images-idx3-ubyte.gz")
    y_total = read_idx("./MNIST/t10k-labels-idx1-ubyte.gz")
    N_train, N_test = 5000, 1000
    x_train, y_train = x_total[:N_train], y_total[:N_train]
    x_test, y_test = x_total[N_train:N_train + N_test], y_total[N_train:N_train + N_test]

    # PCA
    x_train_flattened = np.array(x_train.reshape([N_train, 28 * 28, 1]), dtype=np.float32)
    R = np.einsum("ijk, ilk -> ijl", x_train_flattened, x_train_flattened).mean(axis=0)
    e_vals, e_vecs = np.linalg.eig(R)
    W = e_vecs[:, :50]
    x_train_reduced = W.T.dot(x_train_flattened.reshape([N_train, 28 * 28]).T).T
    x_test_reduced = W.T.dot(np.array(x_test.reshape([N_test, 28*28]).T, dtype=np.float32)).T

    # x1 = np.array(x_test.reshape([100, 28*28, 1]), dtype=np.float32)[50]
    # plt.imshow(x1.reshape([28, 28]), cmap="Greys")
    # plt.show()
    #
    # x1_after = W.dot(W.T).dot(x1).reshape([28, 28])
    # plt.imshow(x1_after, cmap="Greys")
    # plt.show()

    # training SVMs
    model = svm.SVC(C=10, kernel='rbf', gamma=5e-7, decision_function_shape='ovr', cache_size=500)
    model.fit(x_train_reduced, y_train)
    result = model.predict(x_test_reduced)
    print(result)
    print(y_test)
    print(np.where((result-y_test) == 0, 0, 1).mean())

