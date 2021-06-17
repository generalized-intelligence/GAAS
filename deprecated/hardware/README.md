## List of Recommended Hardware for Building Your Own Prototype

In order to facilitate indoor testing and daily maintenance, we did not select models that are too large or too small. Excellent performance and easy maintenance are our original intention to build this drone.

If you wish, we could help you buy all of this hardware and send it to you. Or we can send you the links if you could use taobao.com. Please contact us for more details at adam.wang@gaas.dev

This is a list of hardware that we used for testing. You may use the same hardware for your prototype:

<img src="https://s2.ax1x.com/2019/02/20/k2Jruj.jpg" align="right" width="400px" alt="GA">

### Frame and Power:

  Frame: JMRRC GF360 - One Unit
  
  Motor: T-Motor AIR GEAR 350 - One Unit
  
  ESC: hobywing mini 20A - Four Units
  
  Battery: 4s 5300 25c - Two Units
  
  Charger: ISDT Q6 PRO + power switch - One Unit
  
### Flight control and Sensor:

  Flight control: Micro Pix + Ammeter - One Unit
  
  GPS: U-blox neo M8N + hmc5883l - One Unit
  
  Laser: LIDAR Lite V3 （Optional）
  
  Optic Flow: PX4 FLOW - One Unit
  
  Computational unit: Intel up2 CPU N4200 8G+128G - One Unit
  
  Camera: Occipital Structure Core 3D - One Unit or more
  
### Communication equipment:

  Radio: 900Mhz 100mw - One Unit
  
  Receiver: Frsky XM+ - One Unit
  
  Controller: Frsky X9D+ - One Unit
  
### Miscellaneous
  I2C
  
  XT60
  
  Anti-vibration plate: CC3D 
  
  5V10A DC-DC 

### Related parameter settings for PX4:

EKF2_AID_MASK   Attitude control sensor fusion 

RC_MAP_OFFB_SW  Get into offboard channel  

**We strongly recommend using the remote control's two-stage switch to enter the offboard mode instead of sending commands via TX2, especially if it has not been tested for long periods of time. Use the remote control to avoid accidents.**

SYS_COMPANION   TELEM2 working mode

Usually we need to choose Companion link (921600 baud, 8N1), so that the flight control TELEM2 port works at 921600 baud rate, we need to connect the flight control serial port to the TX2 serial port.

EKF2_HGT_MODE   Height sensor type

By default, we will use Barometric pressure to call the barometer as a height sensor.

We recommend using the Range sensor parameter and purchasing a Lidar lite v3 laser sensor as the height sensor and specifying the port PWM / I2C used by SENS_EN_LL40LS.

(Lidar lite v3 can be powered separately for best results during use)
  
---

hardware 目录放置推荐的硬件配置，方便使用者购买相应的元件制作原型机。

我们的项目完全开源，你可以用任何你希望使用的开发方式来开发 GAAS。

但如果你希望快速上手，直接开始写代码的话，你也可以通过邮件联系我们购买 GAAS 团队内部自己使用的开发套件：adam.wang@gaas.dev



### 我们使用的测试设备明细：

为了方便室内测试和日常维护，我们并没有选取过大或者过小的机型，优秀的性能和便于维护性是我们选配这架无人机的初衷。

机架 猛禽360机架	1	套

好赢 乐天 mini 20A	4	个

T MOTOR Air 350 (不含电调)	1	套

飞控 micro Pix + 电流计	1	套

GPS u blox - M8N    	1	个

数传 900Mhz 100mw	1	对

光流 权盛光流	1	套

intel up2 CPU N4200 8G+128G	1	套

Occipital Structure Core 3D	1	套

5V10A DCDC	1	个

3D 打印摄像头支架	1	个

CC3D 减震	1	套

I2C 分线板	1	套

XT60公母头	4	对

接收机 frsky XM+	1	个

遥控器 Frsky X9D+	1	套

电池 4s 5300 25c	2	块

充电器 ISDT Q6 PRO + 开关电源	1	套


### PX4 相关参数设定：

EKF2_AID_MASK   姿态控制传感器融合  

RC_MAP_OFFB_SW  进入offboard模式的通道  

**我们强烈建议使用遥控器的两段开关进入 offboard 模式而不是通过 TX2 发送命令，尤其是在未经长期测试的条件下。使用遥控器控制进入 offboard 的时机会有效避免意外的发生 。**

SYS_COMPANION   飞控 TELEM2 端口工作模式

通常我们需要选择 Companion link （921600 baud，8N1），使飞控的 TELEM2 端口工作在 921600 波特率下，我们需要连接飞控的串口到TX2的串口。

EKF2_HGT_MODE   高度传感器类型

默认我们将使用 Barometric pressure 来调用气压计作为高度传感器。

我们更建议使用 Range sensor 参数并购买 Lidar lite v3 型激光传感器作为高度传感器，并指定 SENS_EN_LL40LS 使用的端口 PWM / I2C 。

（使用过程中视情况对可对 Lidar lite v3 单独供电以获得最佳效果 ）

2019.04.02 更新

bottom.SLDPRT 和 top.sldprt 为 TX2 + 两组小觅摄像头的壳体工程文件，可使用 solidworks 打开。

初版往往伴随着小 bug 和不合理，希望有经验的朋友可以多指正。

第二版加紧筹备中。
