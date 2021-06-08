import numpy as np
import os
from scipy.spatial.transform import Rotation as R

import json
from collections import OrderedDict


def find_nearest(array, value):
        n = [abs(i-value) for i in array]
        idx = n.index(min(n))
        return idx

def load_img_list(path):
        img_name_list = []
        img_path_list = []
        img_time_list = []

        try:
            f = open(os.path.join(path, 'rgb.txt'), 'r')
        except:
            print('[ERROR] Cannot find image')
            exit(-1)
        for i, line in enumerate(f.readlines()):
            if '#' in line:
                continue
            else:
                line_split = line.split()
                img_name_list.append(line_split[1])
                img_path_list.append(os.path.join(path, line_split[1]))
                img_time_list.append(float(line_split[0]))
        f.close()
        return img_name_list, img_path_list, img_time_list

def load_dep_list(path):
        dep_name_list = []
        dep_path_list = []
        dep_time_list = []

        try:
            f = open(os.path.join(path, 'depth.txt'), 'r')
        except:
            print('[WARNING] Cannot find depth')
            return dep_name_list, dep_path_list

        for i, line in enumerate(f.readlines()):
            if '#' in line:
                continue
            else:
                line_split = line.split()
                dep_name_list.append(line_split[1])
                dep_path_list.append(os.path.join(path, line_split[1]))
                dep_time_list.append(float(line_split[0]))
        f.close()
        return dep_name_list, dep_path_list, dep_time_list

def load_gt_list(path):
        gt_time_list = []
        tx = []
        ty = []
        tz = []
        qx = []
        qy = []
        qz = []
        qw = []

        try:
            f = open(os.path.join(path, 'groundtruth.txt'), 'r')
        except:
            print('[WARNING] Cannot find groundtruth')
            return gt_time_list, tx, ty, tz, qx, qy, qz, qw

        for i, line in enumerate(f.readlines()):
            if '#' in line:
                continue
            else:
                line_split = line.split()

                gt_time_list.append(float(line_split[0]))
                tx.append(float(line_split[1]))
                ty.append(float(line_split[2]))
                tz.append(float(line_split[3]))
                qx.append(float(line_split[4]))
                qy.append(float(line_split[5]))
                qz.append(float(line_split[6]))
                qw.append(float(line_split[7]))

        f.close()
        return gt_time_list, tx, ty, tz, qx, qy, qz, qw

def write_index(img_time_list, dep_time_list, gt_time_list):

    f=open("index.txt", 'w')

    f.write("# all index\n")
    f.write("# file: 'rgbd_dataset_freiburg1_xyz.bag\n")
    f.write("# img_time_list dep_time_list gt_time_list\n")

    for i, v in enumerate(img_time_list):

        d=find_nearest(dep_time_list, v)
        g=find_nearest(gt_time_list, v)

        data='{} {} {}\n'.format(i, d, g)
        f.write(data)

    f.close()

def load_index(path):
        gt_time_list = []
        tx = []
        ty = []
        tz = []
        qx = []
        qy = []
        qz = []
        qw = []

        gt_time_list, tx, ty, tz, qx, qy, qz, qw = load_gt_list(path)

        homo = [[0,0,0,1]]
        extrinsic = []

        try:
            f = open(os.path.join(path, 'index.txt'), 'r')
        except:
            print('[WARNING] Cannot find index')
            return extrinsic

        for i, line in enumerate(f.readlines()):
            if '#' in line:
                continue
            else:
                line_split = line.split()
                index = int(line_split[2])
                # 여기에 그 부분 진행

                t=[[tx[index]], [ty[index]], [tz[index]]]
                r=R.from_quat([qx[index], qy[index], qz[index], qw[index]])

                r=r.as_matrix()

                rc = -r.dot(t)
                rt = np.append(r, rc, axis=1)

                ex = np.append(rt, homo, axis=0)

                extrinsic.append(ex)

        f.close()
        return extrinsic

def write_json(name, extrinsic, f_x, f_y, c_x, c_y):

    file_data = OrderedDict()

    file_data["c_y"] = c_y
    file_data["c_x"] = c_x
    file_data["extrinsic"] = extrinsic
    file_data["f_x"] = f_x
    file_data["f_y"] = f_y

    with open(name, 'w', encoding="utf-8") as make_file:
        json.dump(file_data, make_file, ensure_ascii=False)

# index 저장하기
def main1():
    path=''

    img_name_list=[]
    img_path_list=[]
    img_time_list=[]

    dep_name_list=[]
    dep_path_list=[]
    dep_time_list=[]

    gt_time_list=[]
    tx=[]
    ty=[]
    tz=[]
    qx=[]
    qy=[]
    qz=[]
    qw=[]

    img_name_list, img_path_list, img_time_list=load_img_list(path)
    dep_name_list, dep_path_list, dep_time_list=load_dep_list(path)
    gt_time_list, tx, ty, tz, qx, qy, qz, qw=load_gt_list(path)

    write_index(img_time_list, dep_time_list, gt_time_list)

# 저장한 index에서 json 만들기
def main2():

    f_x = 525.0
    f_y = 525.0
    c_x = 319.5
    c_y = 239.5

    extrinsic = []
    path = ''
    extrinsic = load_index(path)

    for i, ext in enumerate(extrinsic):
        print('iteration : {}'.format(i))
        name = 'pose/' + str(i).zfill(4) + '.json'
        write_json(name, extrinsic[i].tolist(), f_x, f_y, c_x, c_y)

# main1()

main2()