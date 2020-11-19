from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import sys
import os
import numpy as np

class VelocityDataset(Dataset):
    def __init__(self, csv_file, gt_file, transform=None):
        """
        Args:
            csv_file (string): Path to the csv file with annotations.
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.velocity_frame = pd.read_csv(csv_file)
        self.ref_velocity_frame = pd.read_csv(gt_file)
        self.transform = transform

    def __len__(self):
        return len(self.velocity_frame)

    def __getitem__(self, idx):
        velocities = self.velocity_frame.iloc[idx, :]
        # TODO: change ref to ensure the idx window is the same
        ref_velocities = self.ref_velocity_frame.iloc[idx, :]
        v_sample = np.array(velocities)
        v_gt = np.array(ref_velocities)

        if self.transform:
            v_sample = self.transform(v_sample)

        return v_sample, v_gt

class SimpleNetwork(nn.Module):
    def __init__(self, imu_freq=50, gps_freq=1):
        super().__init__()
        
        # Inputs to hidden layer linear transformation
        # self.linear1 = nn.Linear(imu_freq * 3, 3)
        self.linear1 = nn.Linear(3, 3)
        self.sigmoid1 = nn.Sigmoid()
        self.linear2 = nn.Linear(3, 3)
        self.sigmoid2 = nn.Sigmoid()
        self.linear3 = nn.Linear(3, 3)
        self.sigmoid3 = nn.Sigmoid()
        
    def forward(self, x):
        # Pass the input tensor through each of our operations
        x = self.linear1(x)
        x = self.sigmoid1(x)
        x = self.linear2(x)
        x = self.sigmoid2(x)
        x = self.linear3(x)
        x = self.sigmoid3(x)
        
        return x

    def train(self, sample):
        return self.forward(sample.float())


v_path = os.path.join(sys.path[0], "..","data", "imu_vs", "velocity Sun Nov 15 17_52_12 2020.csv")   
dataset = VelocityDataset(v_path, v_path)
dataloader = DataLoader(dataset, batch_size=1,
                        shuffle=True)


model = SimpleNetwork()
# Define the loss
criterion = nn.MSELoss()
# Optimizers require the parameters to optimize and a learning rate
optimizer = optim.Adam(model.parameters(), lr=0.0001)
# optimizer = optim.SGD(lr=0.001)
epochs = 1000
for e in range(epochs):
    running_loss = 0
    for sample, gt in dataloader:    
        # Training pass
        optimizer.zero_grad()
        
        output = model.train(sample)
        loss = criterion(output, gt.float())
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
    else:
        print(f"Training loss: {running_loss/len(dataloader)}")