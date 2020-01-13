import torch
from torch import nn
from torch import tensor
import numpy as np

def generate_data(n, k, b, low=0.0, high=5.0):
    step = (high - low) / n
    x = torch.zeros(n, 1)
    y = torch.zeros(n, 1)
    for i in range(n):
        x[i] = low + step * i
        y[i] = x[i] * k + b + np.random.normal(0.0, 0.05 * (high - low))
    return x, y


class Model(nn.Module):
    def __init__(self):
        """
        In the constructor we instantiate two nn.Linear module
        """
        super(Model, self).__init__()
        self.linear = torch.nn.Linear(1, 1)

    def forward(self, x):
        """
        In the forward function we accept a Variable of input data and we must return
        a Variable of output data. We can use Modules defined in the constructor as
        well as arbitrary operators on Variables.
        """
        y_pred = self.linear(x)
        return y_pred


# our model
model = Model()


# Construct our loss function and an Optimizer. The call to model.parameters()
# in the SGD constructor will contain the learnable parameters of the two
# nn.Linear modules which are members of the model.
criterion = torch.nn.MSELoss(reduction='sum')
optimizer = torch.optim.SGD(model.parameters(), lr=0.001)

# generate data
x_data, y_data = generate_data(100, 2, 1)
print("x_data", x_data)
print("y_data", y_data)

# Training loop
for epoch in range(500):
    # 1) Forward pass: Compute predicted y by passing x to the model
    y_pred = model(x_data)

    # 2) Compute and print loss
    loss = criterion(y_pred, y_data)
    print(f'Epoch: {epoch} | Loss: {loss.item()}')

    # Zero gradients, perform a backward pass, and update the weights.
    optimizer.zero_grad()  
    loss.backward()
    optimizer.step()

# After training
var = tensor([[4.0]])
y_pred = model(var)
print("Prediction (after training)",  4, model(var).data[0][0].item())