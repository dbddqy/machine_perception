import numpy as np
import random
from matplotlib import pyplot as plt


def generate_data(n, p=1.0):
    # p: percentage of data for training

    x11 = np.random.multivariate_normal([4., 3.], [[.08, 0.], [0., .05]], int(0.5*n))
    x12 = np.random.multivariate_normal([2., -2.], [[.05, 0.], [0., .1]], int(0.25*n))
    x13 = np.random.multivariate_normal([7., -4.], [[.1, 0.], [0., .05]], int(0.25*n))
    x1 = np.vstack((x11, x12, x13))
    # x1 = np.random.multivariate_normal([4., 3.], [[2., 0.], [0., .5]], n)  # simple case

    plt.scatter(x1.T[0], x1.T[1], color="red")
    x2 = np.random.multivariate_normal([6., 0.], [[.8, .05], [.05, .2]], n)
    plt.scatter(x2.T[0], x2.T[1], color="blue")
    plt.show()
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


def distance(x1, x2):
    return np.sum((x1-x2)*(x1-x2))


if __name__ == "__main__":
    n = 60
    x_train, y_train, x_test, y_test = generate_data(n)
    K = 4

    # initialization
    center_indices, cluster_indices = [], []
    for i in range(K):
        center_indices.append(random.randint(0, len(x_train)-1))
        cluster_indices.append([])

    flag = True
    while flag:
        # assigning label for all the samples
        for i in range(len(x_train)):
            sample = x_train[i]
            nearest_label = 0
            for label in range(K):
                center_index = center_indices[label]
                if distance(sample, x_train[center_index]) < distance(sample, x_train[center_indices[nearest_label]]):
                    nearest_label = label
            cluster_indices[nearest_label].append(i)
        # computing the new center
        flag = False
        for i in range(K):
            pos_mean = np.mean(x_train[cluster_indices[i]], axis=0)
            nearest_index = 0
            for index in range(len(x_train)):
                if distance(pos_mean, x_train[index]) < distance(pos_mean, x_train[nearest_index]):
                    nearest_index = index
            if center_indices[i] != nearest_index:
                flag = True
            center_indices[i] = nearest_index

    # plot data
    for i in range(K):
        plt.scatter(x_train[cluster_indices[i]].T[0], x_train[cluster_indices[i]].T[1])
    plt.show()
