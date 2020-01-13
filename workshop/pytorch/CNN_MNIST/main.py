from torch import nn, optim, cuda
from torch.utils import data
import torch.nn.functional as F
from torchvision import datasets, transforms
import time
import numpy as np
from matplotlib import pyplot as plt

# training settings
batch_size = 64
device = 'cuda' if cuda.is_available() else 'cpu'
print(f'Training MNIST Model on {device} \n{"=" * 44}')

# MNIST Dataset
train_dataset = datasets.MNIST('./data',
                               train=True,
                               transform=transforms.ToTensor(),
                               download=True)
test_dataset = datasets.MNIST('./data',
                              train=False,
                              transform=transforms.ToTensor(),
                              download=True)

# data loader
train_loader = data.DataLoader(train_dataset,
                               batch_size,
                               shuffle=True)
test_loader = data.DataLoader(test_dataset,
                              batch_size,
                              shuffle=False)


# build neural networks
class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 10, kernel_size=5)
        self.conv2 = nn.Conv2d(10, 20, kernel_size=5)
        self.mp = nn.MaxPool2d(kernel_size=2)
        self.fc = nn.Linear(320, 10)

    def forward(self, x):
        in_size = x.size(0)
        x = F.relu(self.mp(self.conv1(x)))
        x = F.relu(self.mp(self.conv2(x)))
        x = x.view(in_size, -1)
        x = self.fc(x)
        return F.log_softmax(x)


# my model
model = Net()
model.to(device)
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=1e-2, momentum=0.5)


# training step
def train(epoch):
    model.train()  # set model to training mode
    for batch_idx, (data, target) in enumerate(train_loader, 0):
        # send to gpu
        data, target = data.to(device), target.to(device)
        # training: zero grad -> calculate loss -> back propagate -> go one step
        optimizer.zero_grad()
        output = model(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx % 10 == 0:
            print('Train Epoch: {} | Batch Status: {}/{} ({: .0f}%) | Loss: {: .06f}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                       100 * batch_idx / len(train_loader), loss.item()))


# testing step
def test():
    model.eval()  # set model to evaluation mode
    test_loss = 0
    correct = 0
    for data, target in test_loader:
        data, target = data.to(device), target.to(device)
        output = model(data)
        loss = criterion(output, target)
        test_loss += loss.item()
        pred = output.data.max(1, keepdim=True)[1]
        correct += pred.eq(target.data.view_as(pred)).cpu().sum()

    test_loss /= len(test_loader.dataset)
    print(
        f'========================\nTest set: Average Loss: {test_loss:.4f}, Accuracy: {correct}/{len(test_loader.dataset)}'
        f'({100. * correct / len(test_loader.dataset):.0f}%)')


if __name__ == "__main__":
    since = time.time()
    for epoch in range(0, 9):
        epoch_start = time.time()
        train(epoch)
        m, s = divmod(time.time() - epoch_start, 60)
        print(f'Training time: {m:.0f}m {s:.0f}s')
        test()
        m, s = divmod(time.time() - epoch_start, 60)
        print(f'Testing time: {m:.0f}m {s:.0f}s')
    m, s = divmod(time.time() - since, 60)
    print(f'Total Time: {m:.0f}m {s:.0f}s\nModel was trained on {device}')