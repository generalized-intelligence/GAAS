#encoding=utf-8
import os
import sys




image_in_dir = sys.argv[1]
image_out_dir = sys.argv[2]


if __name__ == '__main__':
    img_id_list = []
    i = 0
    for filename in os.listdir(image_in_dir):
        if filename.find('.png')>0:
            fileid = int(filename.split('.')[0])
            img_id_list.append(fileid)
    for fileid in sorted(img_id_list):
        cmd = 'cp {0}/{1}.png {2}/image{3}.png'.format(image_in_dir,fileid,image_out_dir,i)
        os.system(cmd)
        i+=1



