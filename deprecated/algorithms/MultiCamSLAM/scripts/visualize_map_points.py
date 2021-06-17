#encoding=utf-8
#可视化地图点用于debug.

import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 
import numpy as np


'''
	p3d原始数据里有:
		1.x,y,z
		2.frame_id
		3.map_point_id(landmark index)
'''

m_id_to_records = {}

def parse_log_p3d(line):
    if(line.find("xxxx")>=0):
        x,y,z = #匹配p3d.
        frame_id = 
        map_point_id = 
        return (x,y,z),frame_id,map_point_id
    else:
        return None

def parse_lines(log_file):
    l = log_file.readlines()
    res_list = map(parse_log_p3d,l)
    for res in res_list:
        if(res):
            pos,f_id,m_id = res
            if(m_id_to_records.has_key(m_id)):
                m_id_to_records[m_id].append((pos,f_id))
            else
                m_id_to_records[m_id] = [(pos,f_id)]


def draw_matching(ax):
    for m_id in m_id_to_records:
        if len(m_id_to_records[m_id])>=2:
            #有多重记录.展开.
            recs = m_id_to_records[m_id]
            x_,y_,z_ = [],[],[]
            for rec in recs:
                #分别绘制并连接.
                x_.append(rec[0][0])
                y_.append(rec[0][1])
                z_.append(rec[0][2])
            ax.scatter(xs, ys, zs, c='y')
            

if __name__ == '__main__':
    print ('usage: visualize_map_points.py [LOG_FILE]')
    log_file_path = sys.argv[1]
    with open(log_file_path,'r') as f:
        parse_lines(f)
    #显示 m_id_to_records 中的内容.
    ax = plt.subplot(111, projection='3d') # 创建一个三维的绘图工程
    draw_matching(ax)



