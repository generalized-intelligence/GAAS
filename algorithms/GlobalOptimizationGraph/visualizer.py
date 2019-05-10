#encoding=utf-8
import os
import matplotlib.pyplot as plt
import numpy as np

#check_file='without_gps_log.log'
def get_position_by_lines(line):
    if line.find('[GPS_INFO]')>=0 and line.find("GPS_relative_pos")>0:
        print line
        print line.split(':')[-1].split(',')
        return map(float,line.split(':')[-1].split(',')[-4:])
    return None

def get_checkfile(check_file_name):
    return open(check_file_name).readlines()

def get_check_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = get_position_by_lines(line)
        if res:
            xs.append(res[0])
            ys.append(res[1])
            zs.append(res[2])
            #ts.append(res[3])
    return xs,ys,zs#,ts

def get_check_gps_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = map(float,line.split(','))
        xs.append(res[1])
        ys.append(res[2])
        zs.append(res[3])
        ts.append(res[0])
    return xs,ys,zs,ts



#check_file_without_gps = get_checkfile('standard.log')
    

check_file_with_gps = get_checkfile('./log.log') # read gps and visualize.


#xs,ys,zs,ts = get_check_gps_data(check_file_with_gps)
xs,ys,zs = get_check_data(check_file_with_gps)
print len(xs)

plt.scatter(xs, ys, alpha=0.6)  # 绘制散点图，透明度为0.6（这样颜色浅一点，比较好看）

'''
plt.figure()
plt.plot(xs, 'r')
plt.plot(ys, 'y')
plt.plot(zs, 'b')
plt.plot(xsgps,'r.')
plt.plot(ysgps,'y.')
plt.plot(zsgps,'b.')
'''
#plt.plot(xs_gt_al,'r.')
#plt.plot(ys_gt_al,'y.')
#plt.plot(zs_gt_al,'b.')
plt.show()


