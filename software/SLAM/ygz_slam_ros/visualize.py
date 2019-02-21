import os
import matplotlib.pyplot as plt
import numpy as np

#check_file='without_gps_log.log'
def get_position_by_lines(line):
    if line.find('position')>=0:
        print line
        return map(float,line.split()[-1].split(',')[-4:])
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
            ts.append(res[3])
    return xs,ys,zs,ts

def get_check_gps_data(check_file):
    xs = []
    ys = []
    zs = []
    ts = []
    for line in check_file:
        res = get_position_by_lines(line)
        if res:
            xs.append(res[1])
            ys.append(res[2])
            zs.append(res[3])
            ts.append(res[0])
    return xs,ys,zs,ts



#check_file_without_gps = get_checkfile('standard.log')
    

check_file_without_gps = get_checkfile('/home/gi/ros_bag_to_dataset/xyz_by_gps.csv') # read gps and visualize.


xs,ys,zs,ts = get_check_data(check_file_without_gps)

#check_file_with_gps = get_checkfile('cam_noised_no_gps.log')
#check_file_with_gps = get_checkfile('cam_noised_imu_noised_no_gps.log')
check_file_with_gps = get_checkfile('cam_noised_imu_noised_with_gps.log')
xsgps,ysgps,zsgps,tsgps = get_check_data(check_file_with_gps)
xsgps,ysgps,zsgps,tsgps = map(lambda x:np.array(x),[xsgps,ysgps,zsgps,tsgps])


#get ground_truth.
file_ground_truth = '/home/gi/Downloads/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv'
file_ground_truth = open(file_ground_truth).readlines()[1:]
xs_gt = []
ys_gt = []
zs_gt = []
t_gt = []
def get_gt_position_by_line(line):
    if len(line.split(','))>3:
        return map(float,line.split(',')[:4])
    return None

poses=[]
for line in file_ground_truth:
    res = get_gt_position_by_line(line)
    if res:
        xs_gt.append(res[1])
        ys_gt.append(res[2])
        zs_gt.append(res[3])
        t_gt.append(res[0])
def align_time():
    output_list = []
    index_gt = 0
    for index,t in enumerate(ts):
        while True:
            if len(t_gt)<=index_gt:
                break
            if t_gt[index_gt]/1000000000>t:
                print t_gt[index_gt]/1000000000,t
                index_gt += 1
                output_list.append(index_gt)
                break
            else:
                index_gt += 1
    def get_list_by_index_list(l,index_list):
        l_ = []
        for item in index_list:
            l_.append(l[item])
        return l_
    return map(lambda x:get_list_by_index_list(x,output_list),[xs_gt,ys_gt,zs_gt])
xs_gt_al,ys_gt_al,zs_gt_al = align_time()




        
        


xs,ys,zs,xs_gt_al,ys_gt_al,zs_gt_al = np.array(xs),np.array(ys),np.array(zs),np.array(xs_gt_al),np.array(ys_gt_al),np.array(zs_gt_al)


'''
def process_list(l):
    i = 0 
    l_ = []
    for item in l:
        if i%10 ==0:
            l_.append(item)
        i+=1
    return l_
xs_gt_reduced,ys_gt_reduced,zs_gt_reduced = map(process_list,[xs_gt,ys_gt,zs_gt])

for i, line in enumerate(file[:-5]):
    if len(line.split()) > 3:
        if line.split()[-3] == 'returns,':

            x = file[i+1].split()[3]
            y = file[i+2].split()[3]
            z = file[i+3].split()[3]
            x = change(x)
            y = change(y)
            z = change(z)
            #xs.append(x)
            #ys.append(y)
            #zs.append(z)
            xs.append(float(x))
            ys.append(float(y))
            zs.append(float(z))
'''            
print(np.shape(xs))
print(np.shape(ys))
print(np.shape(zs))
print(np.shape(xs_gt_al))

plt.figure()
plt.plot(xs, 'r')
plt.plot(ys, 'y')
plt.plot(zs, 'b')
'''
plt.plot(xsgps,'r.')
plt.plot(ysgps,'y.')
plt.plot(zsgps,'b.')
'''
#plt.plot(xs_gt_al,'r.')
#plt.plot(ys_gt_al,'y.')
#plt.plot(zs_gt_al,'b.')
plt.show()


