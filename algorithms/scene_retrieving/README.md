Retrieving a scene means to find a similar scene from a 3d model or map by loop closing algorithm.
Once similar 2d images has been retrieved,a 3d point mapping and scale solving task shall be done; finally the pos and attitude of camera can be resolved.

Required External Modules:

1.OpenSfM.(a little change has been introduced for feature file output.) located under /algorithms/sfm

    To get ready for usage:
        pip install -r requirements.txt
        python setup.py install

2.nlohmann json.
    
    No need for configuring.

3.Loop Closing. located under /algorithms/loop\_closing.
    
    mkdir build && cd build&&cmake ..&&make -j4

4.roseus.  sudo apt install ros-xxx-roseus


"场景重定位"功能可以支持通过视觉特征,重新在已知场景中通过回环检测等方法定位摄像机的位置.
相似的图像被检索到之后,将会启动一次三维匹配.最终相机的姿态和位置将会被求解.
依赖外部模块:

1.OpenSfM(特征文件输出方式稍有更改.) 在 /algorithms/sfm目录下.
   
    配置方法为在该目录下依次执行:
        pip install -r requirements.txt
        python setup.py install

2.nlohmann json.
    
    直接编译scene_retreving模块即可.

3.Loop Closing. 在algorithms/loop\_closing目录.
    
    mkdir build && cd build&&cmake ..&&make -j4



