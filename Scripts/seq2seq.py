import torch
from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import sys
import os
import numpy as np
import json
import datetime

from odometry import *
from utils import *

class PositionDataset(Dataset):
    """Initialize dataset for xyz velocities

        :param tag_file (string): Path to the csv file with annotations.\n
        :param imu_vs_directory (string): Directory to imu velocities.\n
        :param gps_vs_directory (string): Directory to gps velocities.\n
        :param hz (int): sample frequency of imu, doubles as window_size (true while gps sample rate is 1 Hz).\n
        :param transform (callable, optional): Optional transform to be applied on a sample.
    """
    def __init__(self, dataset_csv, hz=200):
        # open json file specifying which velocity files to use
        self.hz = hz
        full_dataset_csv_path = os.path.join(get_basepath(), "resources", dataset_csv)
        dataset_df = pd.read_csv(full_dataset_csv_path, header=None)
        self.dataset_list = []

        # extract good files into dataset and add to list of pos names
        for i in range(0, len(dataset_df)):
            png_name, yes_no = tuple(dataset_df.iloc[i])
            if yes_no.lower().strip(" ") == "yes":
                # if png is labelled as "good", add name to self.dataset_list
                base_name = png_name.strip(".png")
                self.dataset_list.append(base_name)
    
        # set base dirs for imu and gps pos
        self.imu_pos_directory = os.path.join(get_basepath(), "data", "imu_pos")
        self.gps_pos_directory = os.path.join(get_basepath(), "data", "gps_pos")
        self.hz = hz

        # set file suffixes for imu and gps pos
        self.imu_pos_suffix = "_imu_pos.csv"
        self.gps_pos_suffix = "_gps_pos.csv"

        self.imu_positions = [] # (num_samples, window_size, num_axis) where window_size = hz
        self.gps_positions = [] # (num_samples, num_axis)

        # for each velocity csv file, extract imu and gps velocity data
        for basename in self.dataset_list:
            imu_csv = pd.read_csv(os.path.join(self.imu_pos_directory, basename + self.imu_pos_suffix))
            gps_csv = pd.read_csv(os.path.join(self.gps_pos_directory, basename + self.gps_pos_suffix))
            imu_position = []
            gps_position = []
            for i in range(0, len(gps_csv)):
                try:
                    # for each xyz imu velocity window, save the corresponding gps xyz measurement
                    imu_window = np.array(imu_csv[i*self.hz:i*self.hz+self.hz]) # 200 x 2
                    gps_window = np.expand_dims(np.array(gps_csv.iloc[i]), axis=0) # 1 x 2
                    imu_position.append(imu_window)
                    gps_position.append(gps_window)
                except:
                    pass
            self.imu_positions.append(imu_position)
            self.gps_positions.append(gps_position)

    def __len__(self):
        return len(self.gps_positions)

    def __getitem__(self, idx):

        return self.imu_positions[idx], self.gps_positions[idx]

class DeadReckoningModel(nn.Module):
    """Initialize, train, and evaluate neural network to predict imu velocities

        :basepath: Path to the csv file with annotations.\n
        :imu_freq (optional): imu frequency, default=50\n
        :gps_freq (optional): gps frequency.
    """
    def __init__(self, dataset_csv, imu_freq=200, gps_freq=1):
        super().__init__()
        
        self.basepath = get_basepath()
        self.imu_freq = imu_freq
        self.gps_freq = gps_freq
        self.dataset_csv = dataset_csv

        # # define layer sizes
        # self.input_layer_size = self.imu_freq * 3
        # self.hidden_layer_input_size = int(self.imu_freq * 2)
        # self.hidden_layer_output_size = int(self.imu_freq)
        # self.output_layer_size = 3

        # # define layers
        # self.input_layer = nn.Linear(self.input_layer_size, self.hidden_layer_input_size)
        # self.activation1 = nn.Sigmoid()
        # self.hidden_layer = nn.Linear(self.hidden_layer_input_size, self.hidden_layer_output_size)
        # self.activation2 = nn.Sigmoid()
        # self.output_layer = nn.Linear(self.hidden_layer_output_size, self.output_layer_size)
        # self.activation3 = nn.ReLU()

        # define dataloaders for training and testing
        # self.imu_vs_directory = os.path.join(basepath, "data", "imu_vs")
        # self.gps_vs_directory = os.path.join(basepath, "data", "gps_vs")
        # self.train_tag_file = os.path.join(self.basepath, "resources", "train_tag_files.json")
        # self.test_tag_file = os.path.join(self.basepath, "resources", "test_tag_files.json")

        self.train_dataset = PositionDataset(self.dataset_csv)
        self.train_dataloader = DataLoader(self.train_dataset, batch_size=1,
                                shuffle=True)

        # Define the loss
        self.criterion = nn.MSELoss()
        # define optimizer
        self.optimizer = optim.Adam(self.parameters(), lr=0.001)
        # define num epochs
        self.epochs = 10
        
    def forward(self, x):
        """Pass the input tensor through each layer to produce output"""
        x = self.input_layer(x)
        x = self.activation1(x)
        x = self.hidden_layer(x)
        x = self.activation2(x)
        x = self.output_layer(x)
        x = self.activation3(x)
        
        return x

    def train_sample(self, sample):
        """train a single sample"""
        return self.forward(sample.float())

    def train(self):
        """train model for multiple epochs"""
        for e in range(self.epochs):
            running_loss = 0
            for sample, gt in self.train_dataloader:    
                # Training pass
                self.optimizer.zero_grad()
                output = self.train_sample(sample)
                loss = self.criterion(output, gt.float())
                loss.backward()
                self.optimizer.step()
                
                running_loss += loss.item()
            else:
                print(f"Epoch: {e} Training loss: {running_loss/len(self.train_dataloader)}")
    
    def evaluate(self):
        """evaluate model on paths specified in self.test_tag_file"""

        with open(self.test_tag_file, "r") as jsonfile:
            tagged_paths = json.load(jsonfile)

        for filename, description in tagged_paths.items():
            # for each file specified in the test tag file, compute position using odometry
            basename = filename.split(".gpx")[0]


            # extract velocities for raw imu velocities and gps velocities
            imu_filename = os.path.join(self.imu_vs_directory, basename + self.train_dataset.imu_vs_suffix)
            gps_filename = os.path.join(self.gps_vs_directory, basename + self.train_dataset.gps_vs_suffix)
            raw_imu_vxs, raw_imu_vys, raw_imu_vzs = get_vs_from_file(imu_filename)
            gps_vxs, gps_vys, gps_vzs = get_vs_from_file(gps_filename)

            # consolidate xyz velocities for raw imu velocities
            raw_imu_vs = np.stack([np.array(raw_imu_vxs), np.array(raw_imu_vys), np.array(raw_imu_vzs)]).T

            imu_vs = None

            for i in range(0, len(raw_imu_vxs) - self.imu_freq):
                try:
                    # pass raw imu velocities into neural network and save these output velocities to imu_vs
                    raw_sample = raw_imu_vs[i:i + self.imu_freq, :]
                    sample = np.concatenate([raw_sample[:, 0], raw_sample[:, 1], raw_sample[:, 2]], axis=0)
                    output = self.train_sample(torch.from_numpy(sample))
                    if imu_vs is None:
                        imu_vs = np.expand_dims(output.detach().numpy(), axis=0)
                    else:  
                        imu_vs = np.concatenate([imu_vs, np.expand_dims(output.detach().numpy(), axis=0)], axis=0)
                except:
                    pass

            # compute xyz positions from computed velocities. 
            raw_imu_pxs, raw_imu_pys, raw_imu_pzs = get_xyz_poses(raw_imu_vxs, raw_imu_vys, raw_imu_vzs, 1.0 / self.imu_freq)
            imu_pxs, imu_pys, imu_pzs = get_xyz_poses(imu_vs[:, 0], imu_vs[:, 1], imu_vs[:, 2], 1.0 / self.imu_freq)
            gps_pxs, gps_pys, gps_pzs = get_xyz_poses(gps_vxs, gps_vys, gps_vzs, 1.0 / self.gps_freq)         

            # plot xyz positions
            plot3d(
                xyzs=[(raw_imu_pxs, raw_imu_pys, raw_imu_pzs), \
                      (imu_pxs, imu_pys, imu_pzs), \
                      (gps_pxs, gps_pys, gps_pzs)],
                labels=["positions from raw imu velocity curve", \
                        "positions from imu velocity curve", \
                        "positions from gps velocity curve"],
                title=basename
            )

    def save_model(self, modelname):
        if not os.path.exists(os.path.join(self.basepath, "models")):
            os.makedirs(os.path.join(self.basepath, "models"))
        torch.save(self.state_dict(), os.path.join(self.basepath, "models", modelname))

    def load_model(self, modelname):
        self.load_state_dict(torch.load(os.path.join(self.basepath, "models", modelname)))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--dataset_csv', help="The dataset csv file to use, e.g. small_dataset.csv", action="store", required=True)
    parser.add_argument("--train", help="train (default is evaluate)", action="store_true")
    
    args = parser.parse_args()
    train_mode = args.train

    model = DeadReckoningModel(dataset_csv=args.dataset_csv)

    if train_mode:
        model.train()
        # save model with timestamp to avoid overwriting old models
        model_name = "simplemodel_" + str(datetime.datetime.now()).split(".")[0].replace(":", ".")
        model.save_model(model_name)
        print(model_name)
    else:
        model.load_model("simplemodel_2020-12-01 00.16.14")
        model.evaluate()