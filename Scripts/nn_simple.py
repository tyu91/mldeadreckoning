from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import sys
import os
import numpy as np
import json

class VelocityDataset(Dataset):
    def __init__(self, tag_file, imu_vs_directory, gps_vs_directory, hz=50, transform=None):
        """
            :param tag_file (string): Path to the csv file with annotations.
            :param imu_vs_directory (string): Directory to imu velocities.
            :param gps_vs_directory (string): Directory to gps velocities.
            :param hz (int): sample frequency of imu, doubles as window_size (true while gps sample rate is 1 Hz).
            :param transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        with open(tag_file, "r") as jsonfile:
            self.tagged_paths = json.load(jsonfile)
        self.imu_vs_directory = imu_vs_directory
        self.gps_vs_directory = gps_vs_directory
        self.hz = hz

        self.imu_vs_suffix = "_imu_vel_rolling_window.csv"
        self.gps_vs_suffix = "_gps_vel.csv"

        self.imu_velocities = None # (num_samples, window_size, num_axis) where window_size = hz
        self.gps_velocities = None # (num_samples, num_axis)
        for filename, description in self.tagged_paths.items():
            if "bolaji" not in description:
                continue

            basename = filename.split(".gpx")[0]

            imu_filename = os.path.join(self.imu_vs_directory, basename + self.imu_vs_suffix)
            gps_filename = os.path.join(self.gps_vs_directory, basename + self.gps_vs_suffix)

            imu_csv = pd.read_csv(imu_filename)
            gps_csv = pd.read_csv(gps_filename)

            for i in range(0, len(imu_csv), self.hz):
                try:
                    imu_window = np.expand_dims(np.array(imu_csv[i:i + self.hz]), axis=0)
                    gps_window = np.expand_dims(np.array(gps_csv.iloc[i // self.hz, :]), axis=0)
                    if self.imu_velocities is None:
                        self.imu_velocities = imu_window
                        self.gps_velocities = gps_window
                    else:
                        self.imu_velocities = np.concatenate([self.imu_velocities, imu_window], axis=0)
                        self.gps_velocities = np.concatenate([self.gps_velocities, gps_window], axis=0)
                except:
                    pass
        self.transform = transform

    def __len__(self):
        return len(self.imu_velocities)

    def __getitem__(self, idx):
        imu_sample = self.imu_velocities[idx, :, :] # (window_size, axis)
        gps_sample = self.gps_velocities[idx, :] # axis

        if self.transform:
            imu_sample = self.transform(imu_sample)

        return np.concatenate([imu_sample[:, 0], imu_sample[:, 1], imu_sample[:, 2]], axis=0), gps_sample

class SimpleNetwork(nn.Module):
    def __init__(self, imu_freq=50, gps_freq=1):
        super().__init__()
        
        # Inputs to hidden layer linear transformation
        self.input_layer_size = imu_freq * 3
        self.hidden_layer_input_size = int(imu_freq * 2)
        self.hidden_layer_output_size = int(imu_freq * 2)
        self.output_layer_size = 3

        self.input_layer = nn.Linear(self.input_layer_size, self.hidden_layer_input_size)
        self.activation1 = nn.Sigmoid()
        self.hidden_layer = nn.Linear(self.hidden_layer_input_size, self.hidden_layer_output_size)
        self.activation2 = nn.Sigmoid()
        self.output_layer = nn.Linear(self.hidden_layer_output_size, self.output_layer_size)
        self.activation3 = nn.ReLU()
        
    def forward(self, x):
        # Pass the input tensor through each of our operations
        x = self.input_layer(x)
        x = self.activation1(x)
        x = self.hidden_layer(x)
        x = self.activation2(x)
        x = self.output_layer(x)
        x = self.activation3(x)
        
        return x

    def train(self, sample):
        return self.forward(sample.float())

if __name__ == "__main__":


    imu_vs_directory = os.path.join(sys.path[0], "..","data", "imu_vs")
    gps_vs_directory = os.path.join(sys.path[0], "..","data", "gps_vs")
    tag_file = os.path.join(sys.path[0], "..","resources", "tagged_files.json")

    dataset = VelocityDataset(
                                tag_file=tag_file, 
                                imu_vs_directory=imu_vs_directory, 
                                gps_vs_directory=gps_vs_directory,
                                hz=50
                            )
    dataloader = DataLoader(dataset, batch_size=5,
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
            print(f"Epoch: {e} Training loss: {running_loss/len(dataloader)}")