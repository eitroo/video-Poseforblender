import os
import numpy as np


def Infer(file):
    if os.path.splitext(file)[1] != '.mp4':
        return
    originalDir = os.getcwd()

    os.chdir(os.path.join(__path__[0], "inference"))
    os.system("python infer_video_d2.py \
    --cfg COCO-Keypoints/keypoint_rcnn_R_101_FPN_3x.yaml \
    --output-dir output_directory \
    --image-ext mp4 \"" + file + "\"")

    # prepare data custom
    os.chdir(os.path.join(__path__[0], "data"))
    os.system("python prepare_data_2d_custom.py -i ../inference/output_directory/ -o myvideos")

    # runpy
    os.chdir("..")
    os.system(
        "python run.py -d custom -k myvideos -arc 3,3,3,3,3 -c checkpoint --evaluate pretrained_h36m_detectron_coco.bin --render --viz-subject " +
        os.path.basename(file) +
        " --viz-action custom --viz-camera 0 --viz-video " +
        file + " --viz-export output --viz-size 6")
    os.chdir(originalDir)


def GetOutputPath():
    return (os.path.join(__path__[0], "output.npy"))