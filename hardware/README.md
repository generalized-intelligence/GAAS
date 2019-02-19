This repository lists recommended hardwares for building your own prototype.


hardware 目录放置推荐的硬件配置，方便使用者购买相应的元件制作原型机。

下面是我们使用的测试设备明细：

为了方便室内测试和日常维护，我们并没有选取过大或者过小的机型，优秀的性能和便于维护性是我们选配这架无人机的初衷。

机架动力：

   机架：猛禽 360轴距 碳纤维机架 

   电机：T-Motor AIR GEAR 350 电机

   螺旋桨： T-Motor T9545-AB

   电子调速器： hobywing XRotor micro 20A BLHeli 3-4S

飞行控制设备及传感器：
 
   飞行控制器： 沈阳迎风科技  Micro Pix

   Gps ：U-blox neo M8N + hmc5883l
   
   电流计：沈阳迎风科技 3-6S 60A电流计
   
   激光：LIDAR Lite V3
   
   光流：PX4 FLOW
   
通讯设备：
    
   电台：CUAV LINK 900mhz 250mw
   
   接收机：Frsky XM+
   
电池： 格瑞普（格式） ACE 3S 5300mah 30C

   该配置在室外气温0-5℃可以飞行约 14 分钟，相信在更好的室外条件下会获得更好的续航表现。 
   
   
当前计划：   
   
1.测试各品牌双目相机的稳定性。
   
2.为 TX2 + TX2 底版 + 两组双目设计 3D 打印（CNC）外壳。 

3.为 TX2 底版做供电优化。

4.在现有的测试机配置基础上优化动力配置及 PID，提供更高效的飞行平台。


关于PX4参数设定：

EKF2_AID_MASK   姿态控制传感器融合  

RC_MAP_OFFB_SW  进入offboard模式的通道  

我们强烈建议使用遥控器的两段开关进入 offboard 模式而不是通过 TX2 发送命令，尤其是在未经长期测试的条件下。使用遥控器控制进入 offboard 的时机会有效避免意外的发生 。

SYS_COMPANION   飞控 TELEM2 端口工作模式

通常我们需要选择 Companion link （921600 baud，8N1），使飞控的 TELEM2 端口工作在 921600 波特率下，我们需要连接飞控的串口到TX2的串口。

EKF2_HGT_MODE   高度传感器类型

默认我们将使用 Barometric pressure 来调用气压计作为高度传感器 ，我们更建议使用Range sensor 参数并购买 Lidar v3 型激光传感器作为高度传感器。

