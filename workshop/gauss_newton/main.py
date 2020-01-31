import numpy as np
from matplotlib import pyplot as plt


def generate_data(n, _theta_0, _model, low=0.0, high=100.0):
    step = (high - low) / n
    x, y = [], []
    for i in range(n):
        x.append(low + step * i)
        y.append(_model(x[i], theta_0) + np.random.normal(0.0, 0.0005 * (high - low)))
    return np.array(x), np.array(y)


def model(x, theta):
    return np.log(x*x*theta[0] + x*theta[1] + theta[2])


def train(_x_train, _y_train, _theta, _model, steps=100):
    cost, last_cost = .0, .0
    _theta_new = _theta.copy()
    for step in range(steps):
        H = np.zeros([3, 3], dtype=np.float32)
        b = np.zeros([3, 1], dtype=np.float32)
        for i in range(_x_train.shape[0]):
            y_est = _model(_x_train[i], _theta_new)
            error = y_est - _y_train[i]
            jacobi = np.zeros([1, 3], dtype=np.float32)
            jacobi[0, 0] = _x_train[i] * _x_train[i] / np.exp(y_est)
            jacobi[0, 1] = _x_train[i] / np.exp(y_est)
            jacobi[0, 2] = 1 / np.exp(y_est)
            H += jacobi.T.dot(jacobi)
            b += -error * jacobi.T
            cost += error * error
        if step > 1 and cost > last_cost:
            break

        dx = np.linalg.solve(H, b)
        _theta_new[0] += dx[0, 0]
        _theta_new[1] += dx[1, 0]
        _theta_new[2] += dx[2, 0]
        print("iteration: {0}, cost: {1}".format(step, cost))
        last_cost = cost
        cost = 0
    return _theta_new


if __name__ == "__main__":
    theta_0 = np.array([0.2, 0.5, 20.0])
    x_train, y_train = generate_data(100, theta_0, model)
    theta = np.array([0.1, 0.0, 10.0])
    # plot x, y data
    plt.scatter(x_train, y_train, color="red")
    # training
    theta_new = train(x_train, y_train, theta, model, steps=1000)
    print(theta_new)
    # plot line
    y_new = model(x_train, theta_new)
    plt.plot(x_train, y_new)

    plt.show()

