This module implements the transformation of basic px4 flight controller status info to GAAS generalized flight controller info.

If you replace px4 with your own flight controller, you need to edit this module to fill in the flight controller status info manually.

You may need to run:

    ./install_geographiclib_datasets.sh

to install the dependencies of mavros. You can find the script at /opt/ros/{$YOUR_ROS_VERSION}/lib/mavros/install_geographiclib_datasets.sh


这里实现px4飞控消息到GAAS通用飞控消息格式的转换与对应msg的转发.

如果你用自定义的飞控替代px4,你需要手动实现这个模块的对应飞控状态信息填充.


可能需要运行

    ./install_geographiclib_datasets.sh

安装mavros必要的依赖项.你可以在/opt/ros/{$YOUR_ROS_VERSION}/lib/mavros/install_geographiclib_datasets.sh 找到这个脚本.
