import torch
import torch.nn as nn
from torchvision import datasets, models, transforms
import cv2
import numpy as np

path = 'detection_model.pt'
model_ft = torch.load(path)
model_ft.eval()

#position of each square
square = {'H8': (569, 639, 371, 440), 'H7': (625, 692, 371, 440), 'H6': (678, 750, 371, 440), 'H5': (736, 807, 371, 440), 'H4': (793, 859, 371, 440), 'H3': (845, 917, 371, 440), 'H2': (903, 971, 371, 440), 'H1': (957, 1027, 371, 440), 'G8': (569, 639, 424, 493), 'G7': (625, 692, 424, 493), 'G6': (678, 750, 424, 493), 'G5': (736, 807, 424, 493), 'G4': (793, 859, 424, 493), 'G3': (845, 917, 424, 493), 'G2': (903, 971, 424, 493), 'G1': (957, 1027, 424, 493), 'F8': (569, 639, 482, 551), 'F7': (625, 692, 482, 551), 'F6': (678, 750, 482, 551), 'F5': (736, 807, 482, 551), 'F4': (793, 859, 482, 551), 'F3': (845, 917, 482, 551), 'F2': (903, 971, 482, 551), 'F1': (957, 1027, 482, 551), 'E8': (569, 639, 536, 608), 'E7': (625, 692, 536, 608), 'E6': (678, 750, 536, 608), 'E5': (736, 807, 536, 608), 'E4': (793, 859, 536, 608), 'E3': (845, 917, 536, 608), 'E2': (903, 971, 536, 608), 'E1': (957, 1027, 536, 608), 'D8': (569, 639, 592, 660), 'D7': (625, 692, 592, 660), 'D6': (678, 750, 592, 660), 'D5': (736, 807, 592, 660), 'D4': (793, 859, 592, 660), 'D3': (845, 917, 592, 660), 'D2': (903, 971, 592, 660), 'D1': (957, 1027, 592, 660), 'C8': (569, 639, 649, 718), 'C7': (625, 692, 649, 718), 'C6': (678, 750, 649, 718), 'C5': (736, 807, 649, 718), 'C4': (793, 859, 649, 718), 'C3': (845, 917, 649, 718), 'C2': (903, 971, 649, 718), 'C1': (957, 1027, 649, 718), 'B8': (569, 639, 702, 772), 'B7': (625, 692, 702, 772), 'B6': (678, 750, 702, 772), 'B5': (736, 807, 702, 772), 'B4': (793, 859, 702, 772), 'B3': (845, 917, 702, 772), 'B2': (903, 971, 702, 772), 'B1': (957, 1027, 702, 772), 'A8': (569, 639, 758, 828), 'A7': (625, 692, 758, 828), 'A6': (678, 750, 758, 828), 'A5': (736, 807, 758, 828), 'A4': (793, 859, 758, 828), 'A3': (845, 917, 758, 828), 'A2': (903, 971, 758, 828), 'A1': (957, 1027, 758, 828)}

processing = transforms.Compose([
        transforms.ToPILImage(),                         
        transforms.Resize(256),
        transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    ])

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

class_names = ['B', 'K', 'N', 'P', 'Q', 'R', '_', 'b', 'k', 'n', 'p', 'q', 'r']
chessboard = np.zeros((8,8),dtype=str)
col = {'A':7,'B':6,'C':5,'D':4,'E':3,'F':2,'G':1,'H':0}
row = {'1':7,'2':6,'3':5,'4':4,'5':3,'6':2,'7':1,'8':0}


# detect the chessboard and return a list of chessboard 
def detecting(image,square,model_ft):

    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
    count = 1
    img_list = []
    key_list = []
    for key,value in square.items():
        if count < 4:
            y1,y2,x1,x2 = value
            img = image[x1:x2,y1:y2,:]
            img_tt = processing(img)
            img_list.append(img_tt)
            key_list.append(key)
            count += 1
        else:
            y1,y2,x1,x2 = value
            img = image[x1:x2,y1:y2,:]
            img_tt = processing(img)
            img_list.append(img_tt)
            key_list.append(key)
            count = 1
            inputs = torch.stack(img_list,0)
            inputs = inputs.to(device)
            outputs = model_ft(inputs)
            _, preds = torch.max(outputs,1)
            for k in key_list:
                chessboard[col[k[0]],row[k[1]]] = class_names[preds[key_list.index(k)]]
            img_list,key_list = [],[]
        
    return chessboard