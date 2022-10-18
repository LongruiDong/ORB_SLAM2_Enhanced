"""
用python读入 map.bin 文件
目的是为了 载入nice-slam
 
"""
import argparse, os, copy
import random
# -*- coding:utf-8 -*-
import numpy as np


def read_map(mapbinfile):
    
    print("read map bin: {}".format(mapbinfile))
    
    mapbin = open(mapbinfile, 'rb')
    
    print('1')






if __name__ == '__main__':
    # parser command lines
    parser = argparse.ArgumentParser(description='''
      
    ''') 
    parser.add_argument('mapbinfile', type=str, help='orb outputed map binary file path', default="result/map_proxy.bin") # result_mapsl/map.bin result/map_proxy.bin 
    # parser.add_argument('rawpcdfile', type=str, help='3d 点文件路径',default="office0_orb_mappts.txt")
    # parser.add_argument('seq', help='所用序列',default='08')

    args = parser.parse_args()
    # 读取参数
    mapbinfile = args.mapbinfile

    # print("\n report result...\n")
    read_map(mapbinfile)