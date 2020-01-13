import torch
from torch.utils import data
from torch import cuda
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
from torchvision import transforms
import numpy as np
import matplotlib.pyplot as plt

# training settings
transform = transforms.Compose(
    [transforms.ToTensor(),
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])
batch_size = 4
device = 'cuda' if cuda.is_available() else 'cpu'
print(f'Training MNIST Model on {device} \n{"=" * 44}')

# CIFAR Dataset
train_dataset = torchvision.datasets.CIFAR10('./data',
                                             train=True,
                                             transform=transform,
                                             download=True)
test_dataset = torchvision.datasets.CIFAR10('/data',
                                            train=True,
                                            transform=transform,
                                            download=True)
train_loader = data.DataLoader(train_dataset,
                               batch_size,
                               shuffle=True,
                               num_workers=0)
test_loader = data.DataLoader(test_dataset,
                              batch_size,
                              shuffle=False,
                              num_workers=0)
classes = ('plane', 'car', 'bird', 'cat', 'deer', 'dog',
           'frog', 'horse', 'ship', 'truck')


# function to show an image
def imshow(img):
    img = img / 2 + 0.5  # unnormalize
    npimg = img.numpy()
    plt.imshow(np.transpose(npimg, (1, 2, 0)))
    plt.show()


# define neural networks
class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(3, 6, kernel_size=5)
        self.conv2 = nn.Conv2d(6, 16, kernel_size=5)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(16 * 5 * 5, 120)
        self.fc2 = nn.Linear(120, 84)
        self.fc3 = nn.Linear(84, 10)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = x.view(-1, 16 * 5 * 5)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x


# my model
model = Net()
model.to(device)

# criterion and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=1e-3, momentum=0.9)


def train(epoch):
    model.train()
    for batch_idx, (data, target) in enumerate(train_loader, 0):
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()
        output = model(data)
        loss = criterion(output, target)
        loss.backward()
        optimizer.step()
        if batch_idx % 100 == 0:
            print('Train Epoch: {} | Batch Status {}/{} ({:.0f}%) | Loss: {}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                100 * batch_idx / len(test_loader), loss.item()))


def test():
    model.eval()
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
    print(f'========================\nTest set: Average Loss: {test_loss:.4f}, Accuracy: {correct}/{len(test_loader.dataset)}'
          f'({100. * correct / len(test_loader.dataset):.0f}%)')


if __name__ == "__main__":
    # training & testing
    for epoch in range(0, 20):
        train(epoch)
        test()
    # save trained model
    PATH = './cifar_net.pth'
    torch.save(model.state_dict(), PATH)
    # load trained model (not necessary here, juet to illustrate how to do so)
    model_to_test = Net()
    model_to_test.load_state_dict(torch.load(PATH))

    # visualization
    # take some random images
    dataiter = iter(test_loader)
    images, labels = dataiter.next()
    # print images
    imshow(torchvision.utils.make_grid(images))
    print('GroundTruth: ', ' '.join('%5s' % classes[labels[j]] for j in range(4)))
    # use model to predict
    pred_outputs = model_to_test(images)
    _, predicted = torch.max(pred_outputs, 1)
    print('Predicted: ', ' '.join('%5s' % classes[predicted[j]]
                                  for j in range(4)))

