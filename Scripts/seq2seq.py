import torch
from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import pandas as pd
import sys
import os
import numpy as np
import json
import datetime
from matplotlib import animation
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm


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
    def __init__(self, train_list, hz=200):
        # open dataset csv specifying what files to use for train and testing
        self.hz = hz
        self.dataset_list = train_list
    
        # set base dirs for imu and gps pos
        self.imu_pos_directory = os.path.join(get_basepath(), "data", "imu_pos")
        self.gps_pos_directory = os.path.join(get_basepath(), "data", "gps_pos")
        self.hz = hz

        # set file suffixes for imu and gps pos
        self.imu_pos_suffix = "_imu_pos.csv"
        self.gps_pos_suffix = "_gps_pos.csv"

        self.imu_diffs = [] # (num_samples, window_size, num_axis) where window_size = hz
        self.gps_diffs = [] # (num_samples, num_axis)

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
                    imu_window = np.hstack([imu_window[:, 0], imu_window[:, 1]]) # if you want it to be 400 (rn using two separate encoders)
                    imu_position.append(imu_window)
                    gps_position.append(gps_window)
                except:
                    pass
            imu_diff = np.array([snd - fst for fst, snd in zip(imu_position[:-1], imu_position[1:])])
            gps_diff = np.array([snd - fst for fst, snd in zip(gps_position[:-1], gps_position[1:])])
            self.imu_diffs.append(imu_diff)
            self.gps_diffs.append(gps_diff)

    def __len__(self):
        return len(self.gps_diffs)

    def __getitem__(self, idx):
        return self.imu_diffs[idx], self.gps_diffs[idx], self.dataset_list[idx]

class Encoder(nn.Module):
    def __init__(self, input_size, emb_size, hidden_size):
        super(Encoder, self).__init__()
        # initialize variables
        self.embedding_size = emb_size # 256 for now
        self.hidden_size = hidden_size # 128 for now
        self.input_size = input_size # 400 (2 x 200 stacked)
    
        # create nn layers
        self.embedding_layer = nn.Linear(self.input_size, self.embedding_size)
        self.lstm = nn.LSTM(self.embedding_size, self.hidden_size, dropout=0.5)

    def forward(self, x):
        x = x.permute(1, 0, 2) # get into shape len * 1 * 400
        linear_output = self.embedding_layer(x.float())
        _, hidden_output = self.lstm(linear_output)
        return hidden_output # of size ([1, 1, 128], [1, 1, 128])


class Decoder(nn.Module):
    def __init__(self, emb_size, hidden_size, output_size):
        super(Decoder, self).__init__()
        # initialize variables
        self.embedding_size = emb_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # create nn layers
        self.embedding_layer = nn.Linear(self.output_size, self.embedding_size)
        self.lstm = nn.LSTM(self.embedding_size, self.hidden_size)
        self.output_layer = nn.Linear(self.hidden_size, self.output_size)

        self.loss_fn = nn.MSELoss()

    def forward(self, hidden_input, sample_length, output_length, ground_truth, beta):
        curr_hidden = hidden_input

        predicted_output = []
        loss = 0
        curr_output = torch.ones([1, 2])
        curr_embedded_output = self.embedding_layer(curr_output).unsqueeze(0)

        gt = ground_truth.squeeze(0) # shape is len x 1 x 2
        used_gt = []

        for t in range(output_length):
            new_output, new_hidden = self.lstm(curr_embedded_output, curr_hidden)
            new_raw_output = self.output_layer(new_output)
            gt_t = gt[t].unsqueeze(0)
            loss += self.loss_fn(new_raw_output, gt_t.float())
            if np.random.binomial(1, beta, 1)[0] == 1:
                used_gt.append(True)
                curr_embedded_output = self.embedding_layer(gt_t.float())
            else:
                used_gt.append(False)
                curr_embedded_output = self.embedding_layer(new_raw_output)
            curr_hidden = new_hidden
            predicted_output.append(new_raw_output.squeeze(0))
        return predicted_output, loss, used_gt


class DeadReckoningModel(nn.Module):
    """Initialize, train, and evaluate neural network to predict imu velocities

        :basepath: Path to the csv file with annotations.\n
        :imu_freq (optional): imu frequency, default=50\n
        :gps_freq (optional): gps frequency.
    """
    def __init__(self, dataset_csv, test_files, beta, train_percent, num_epochs, gps_dropout, anneal_beta, animate, imu_freq=200, gps_freq=1):
        super().__init__()
        
        self.basepath = get_basepath()
        self.imu_freq = imu_freq
        self.gps_freq = gps_freq
        self.dataset_csv = dataset_csv
        self.test_files = test_files
        self.beta = beta
        self.train_percent = train_percent
        self.epochs = num_epochs
        self.gps_dropout = gps_dropout
        self.anneal_beta = anneal_beta
        self.animate = animate

        self.input_size = 400
        self.emb_size = 256
        self.hidden_size = 128
        self.output_size = 2

        self.encoder = Encoder(self.input_size, self.emb_size, self.hidden_size)
        self.encoder_opt = optim.Adam(self.encoder.parameters(), lr=0.001)
        self.decoder = Decoder(self.emb_size, self.hidden_size, self.output_size)
        self.decoder_opt = optim.Adam(self.decoder.parameters(), lr=0.001)

        # split into train and test datasets
        self.train_list = []
        self.test_list = []
        self.split_train_test()

        self.train_dataset = PositionDataset(self.train_list)
        self.train_dataloader = DataLoader(self.train_dataset, batch_size=1,shuffle=True)
        self.test_dataset = PositionDataset(self.test_list)
        self.test_dataloader = DataLoader(self.test_dataset, batch_size=1,shuffle=True)

    def split_train_test(self):
        full_test_files_path = os.path.join(get_basepath(), "resources", self.test_files)
        test_files_set = set()
        with open(full_test_files_path) as test_file:
            lines = test_file.readlines()
            for line in lines:
                test_files_set.add(line.strip())
        
        self.test_list = list(test_files_set)

        full_dataset_path = os.path.join(get_basepath(), "resources", self.dataset_csv)
        dataset_df = pd.read_csv(full_dataset_path, header=None)

        # extract good files into dataset and add to list of pos names
        for i in range(0, len(dataset_df)):
            png_name, _ = tuple(dataset_df.iloc[i])
            base_name = png_name.strip(".png")
            if base_name not in test_files_set:
                self.train_list.append(base_name)

    def diff_to_pos(self, diffs):
        """convert dx,dy to x,y positions
        
        :param diffs: list of dx, dy to convert to a true trajectory
        """
        output_positions = np.zeros([len(diffs) * 2,])
        cx = 0
        cy = 0
        for i in range(len(diffs)):
            diff = diffs[i]
            dx, dy = tuple(diff.squeeze(0))
            cx += dx
            cy += dy

            output_positions[i] = cx
            output_positions[len(diffs) + i] = cy

        return output_positions


    def train(self):
        """train model for multiple epochs"""
        for e in range(self.epochs):
            running_loss = 0
            running_acc = 0
            for sample, gt, _ in self.train_dataloader:  
                self.encoder_opt.zero_grad()
                self.decoder_opt.zero_grad()

                # Training pass
                hidden = self.encoder.forward(sample)
                gt = gt.squeeze(0)
                predicted_output, loss, _ = self.decoder.forward(hidden, len(sample), len(gt), gt, self.beta)
                loss /= len(gt)

                # compute accuracy (l2norm between predicted and ground truth position)
                gt_pos = self.diff_to_pos(gt)
                gt_x, gt_y = np.array(gt_pos[:len(gt)]), np.array(gt_pos[len(gt):])
                gt_xy = np.vstack([gt_x, gt_y]).T
                pred_pos = self.diff_to_pos(predicted_output)
                pred_x, pred_y = np.array(pred_pos[:len(predicted_output)]), np.array(pred_pos[len(predicted_output):])
                pred_xy = np.vstack([pred_x, pred_y]).T

                acc = np.mean(np.linalg.norm(pred_xy - gt_xy, axis=1))

                # update network
                loss.backward(retain_graph=True)
                self.encoder_opt.step()
                self.decoder_opt.step()

                running_loss += loss.item()
                running_acc += acc

                if self.anneal_beta:
                    self.beta = max(0.25, self.beta - (0.75 / self.epochs))
            else:
                print(f"Epoch: {e} Training loss: {running_loss/len(self.train_dataloader)} Training accuracy: {running_acc/len(self.train_dataloader)}")
    '''
    what do i want the animation to do:
    every interval of 1000ms, append gt_x, gt_y, so on and so forth those predictions. and connect them.
    '''
    def animate_evaluation(self, gt_x, gt_y, pred_x, pred_y, imu_x, imu_y, used_gt, title):
        fig = plt.figure()
        c_gt_x, c_gt_y, c_pred_x, c_pred_y, c_imu_x, c_imu_y = [], [], [], [], [], []
        cmap = ['green' if elem else 'red' for elem in used_gt]
        ax1 = plt.axes(
            xlim=(min(min(gt_x), min(pred_x), min(imu_x)) - 10, max(max(gt_x), max(pred_x), max(imu_x)) + 10), \
            ylim=(min(min(gt_y), min(pred_y), min(imu_y)) - 10, max(max(gt_y), max(pred_y), max(imu_y)) + 10))
        line, = ax1.plot([], [], lw=2)
        line = LineCollection([], lw=2)
        ax1.add_collection(line)
        plt.xlabel('x (meters)')
        plt.ylabel('y (meters)')

        plotlays, plotcols, plotlabels = [3], ["black","green", "blue"], ["gt position", "predicted position", "imu position"]
        lines = []
        for index in range(3):
            lobj = ax1.plot([],[],lw=2,color=plotcols[index])[0]
            lines.append(lobj)

        def init():
            for line in lines:
                line.set_data([], [])
            return lines

        def animate(i):
            # c_gt_x.append(gt_x[i])
            # c_gt_y.append(gt_y[i])
            # c_pred_x.append(pred_x[i])
            # c_pred_y.append(pred_y[i])
            # c_imu_x.append(imu_x[i])
            # c_imu_y.append(imu_y[i])
            c_gt_x = [gt_x[i], gt_x[i+1]]
            c_gt_y = [gt_y[i], gt_y[i+1]]
            c_pred_x = [pred_x[i], pred_x[i+1]]
            c_pred_y = [pred_y[i], pred_y[i+1]]
            c_imu_x = [imu_x[i], imu_x[i+1]]
            c_imu_y = [imu_y[i], imu_y[i+1]]

            xlist = [c_gt_x, c_pred_x, c_imu_x]
            ylist = [c_gt_y, c_pred_y, c_imu_y]

            for lnum in range(3):
                if lnum == 1:
                    line, = ax1.plot(xlist[lnum], ylist[lnum], c=cmap[i])
                else:
                    line, = ax1.plot(xlist[lnum], ylist[lnum], c=plotcols[lnum])
                lines.append(line)

            return lines
        
        anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(gt_x)-1, interval=100, repeat=False)
        plt.legend(lines, plotlabels)
        plt.title(title)
        plt.show()

            

    def evaluate(self):
        """evaluate model on paths specified in self.test_tag_file"""
        with torch.no_grad():
            for sample, gt, basename in self.test_dataloader:
                # Training pass
                hidden = self.encoder.forward(sample)
                gt = gt.squeeze(0)
                predicted_output, loss, used_gt = self.decoder.forward(hidden, len(sample), len(gt), gt, 1.0-self.gps_dropout)
                loss /= len(gt)

                # compute accuracy (l2norm between predicted and ground truth position)
                gt_pos = self.diff_to_pos(gt)
                gt_x, gt_y = np.array(gt_pos[:len(gt)]), np.array(gt_pos[len(gt):])
                gt_xy = np.vstack([gt_x, gt_y]).T
                
                sample = sample.squeeze(0)
                sample = np.vstack([sample[:, self.imu_freq-1], sample[:, self.imu_freq*2-1]]).T
                sample = np.expand_dims(sample, axis=1)
                imu_pos = self.diff_to_pos(sample)
                imu_x, imu_y = np.array(imu_pos[:len(sample)]), np.array(imu_pos[len(sample):])
                imu_xy = np.vstack([imu_x, imu_y]).T

                pred_pos = self.diff_to_pos(predicted_output)
                pred_x, pred_y = np.array(pred_pos[:len(predicted_output)]), np.array(pred_pos[len(predicted_output):])
                pred_xy = np.vstack([pred_x, pred_y]).T
                
                acc = np.mean(np.linalg.norm(pred_xy - gt_xy, axis=1))
                imu_acc = np.mean(np.linalg.norm(imu_xy - gt_xy, axis=1))

                # plot the results compared to regular transform
                plot2d(
                        xys = [(gt_x, gt_y), (pred_x, pred_y), (imu_x, imu_y)],
                        labels=["ground truth gps", "predicted position based on imu", "original imu position with transformation"], 
                        title=basename[0],
                        colors=["black", "green", "blue"]
                    )
                if animate:
                    self.animate_evaluation(gt_x, gt_y, pred_x, pred_y, imu_x, imu_y, used_gt, basename[0])
                print(f"{basename[0]}: Test loss: {loss} Test accuracy: {acc} imu accuracy: {imu_acc}")

    def save_model(self, modelname):
        if not os.path.exists(os.path.join(self.basepath, "models")):
            os.makedirs(os.path.join(self.basepath, "models"))
        torch.save(self.state_dict(), os.path.join(self.basepath, "models", modelname))

    def load_model(self, modelname):
        self.load_state_dict(torch.load(os.path.join(self.basepath, "models", modelname)))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--dataset_csv', help="The dataset csv file to use, e.g. small_dataset.csv", action="store", required=True)
    parser.add_argument('--test_files', help="The dataset txt file to use, e.g. small_dataset_test_list.txt", action="store", required=True)
    parser.add_argument("--eval_model", help="specify which model to evaluate dataset with", action="store")
    parser.add_argument("--gps_dropout", help="gps dropout value (default is 0, no dropout)", type=float, default=0.0)
    parser.add_argument("--animate", help="animate gps prediction in realtime", action="store_true")
    
    args = parser.parse_args()
    train_mode = args.eval_model is None

    beta = 1.0 # 1 is teacher forcing, 0 is model only
    train_percent = 0.8 # unused 
    num_epochs = 100
    gps_dropout = args.gps_dropout
    print("gps dropout: ", gps_dropout)
    anneal_beta = False
    animate = args.animate

    model = DeadReckoningModel(
        dataset_csv=args.dataset_csv, 
        test_files=args.test_files, 
        beta=beta, 
        train_percent=train_percent, 
        num_epochs=num_epochs, 
        gps_dropout=gps_dropout, 
        anneal_beta=anneal_beta,
        animate=animate
    )

    if train_mode:
        model.train()
        model.evaluate()
        # save model with timestamp to avoid overwriting old models
        model_name = "seq2seqmodel_" + str(datetime.datetime.now()).split(".")[0].replace(":", ".")
        model.save_model(model_name)
        print(model_name)
    else:
        model.load_model(args.eval_model)
        model.evaluate()