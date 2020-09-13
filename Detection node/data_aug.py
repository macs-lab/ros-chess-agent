import cv2
import imutils
import random
import numpy as np
import os 
import shutil


def rotate_image(img,degree):
    rot = imutils.rotate(img,angle = degree)
    return rot

def contrast_brightness(img):
    rate1 = random.uniform(-0.4,0.4)
    beta = int(100*rate1)
    rate2 = random.uniform(-0.5,0.5)
    alpha = 1+rate2
    new_image = np.zeros(img.shape, img.dtype)
    for y in range(img.shape[0]):
        for x in range(img.shape[1]):
            for c in range(img.shape[2]):
                new_image[y,x,c] = np.clip(alpha*img[y,x,c] + beta, 0, 255)
    return new_image

def add_sp_noise(image):
    amount = random.uniform(0,0.003)
    s_vs_p = 0.5
    out = np.copy(image)
    # Salt mode
    num_salt = np.ceil(amount * image.size * s_vs_p)
    while num_salt > 0:
        x1,y1 = random.randrange(0, img.shape[0]+1), random.randrange(0, img.shape[1]+1)
        x2,y2 = random.randrange(0, img.shape[0]+1), random.randrange(0, img.shape[1]+1)
        cv2.circle(out,(x1,y1),1,(255,255,255),-1)
        cv2.circle(out,(x2,y2),1,(0,0,0),-1)
        num_salt -= 1
    return out

parent_dir = os.getcwd()

train = os.path.join(parent_dir,'train')
val = os.path.join(parent_dir,'val')

os.mkdir(train)
os.mkdir(val)

rot_angle = [0,45,90,135,180,225,270,315]
function = [contrast_brightness,add_sp_noise]
subdirs = [x[0] for x in os.walk(parent_dir) if x[0] != parent_dir and x[0] != train and x[0] != val]
for folder in subdirs:
    os.chdir(folder)
    chess_type = os.path.basename(folder)

    val_folder = os.path.join(val,chess_type)
    os.mkdir(val_folder)
    train_folder = os.path.join(train,chess_type)
    os.mkdir(train_folder) 

    validation = [os.path.join(folder,file) for file in random.sample(os.listdir(),6)]
    for image in validation:
        shutil.move(image, val_folder)
    count = 0
    for file in os.listdir(folder):
        img = cv2.imread(file,1)
        for degree in random.choices(rot_angle,k=3):
            out = img.copy()
            out = rotate_image(out,degree)
            out = random.choice(function)(out)
            out = random.choice(function)(out)
            name = str(count) + '.png'
            count += 1
            cv2.imwrite(os.path.join(train_folder, name),out)
            
            

            


