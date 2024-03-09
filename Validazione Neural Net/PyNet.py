import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from prettytable import PrettyTable

def count_parameters(model):
    table = PrettyTable(["Modules", "Parameters"])
    total_params = 0
    for name, parameter in model.named_parameters():
        if not parameter.requires_grad: continue
        params = parameter.numel()
        table.add_row([name, params])
        total_params+=params
    print(table)
    print(f"Total Trainable Params: {total_params}")
    return total_params




dtype = torch.float64
model = torch.load("../data/px4_controller_trained.pt")
count_parameters(model)

# x=pd.read_csv("./controller_state.csv")
# x=torch.tensor(x.values)
# y=pd.read_csv("./controller_target.csv")
# y=torch.tensor(y.values)

W1=pd.read_csv("../data/W1_05_01_LTI_zero_eq_torch.csv", header=None).to_numpy() #7x9
W2=pd.read_csv("../data/W2_05_01_LTI_zero_eq_torch.csv", header=None).to_numpy() #4x7
B1=pd.read_csv("../data/b1_05_01_LTI_zero_eq_torch.csv", header=None).to_numpy() #7x1
B2=pd.read_csv("../data/b2_05_01_LTI_zero_eq_torch.csv", header=None).to_numpy() #4x1
model.eval()

x = np.random.rand(9) 
x = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9], dtype=np.float64)
hid1 = np.dot(W1, x) + B1.flatten() 
hid1 = np.where(hid1 < 0, 0.01 * hid1, hid1)
out = np.dot(W2, hid1) + B2.flatten()
out_torch=model(torch.tensor(x))
print(f"Custom Net output: {out}")
print("--------------------")
print(f"Pytorch Net output: {out_torch.detach().numpy()}")

