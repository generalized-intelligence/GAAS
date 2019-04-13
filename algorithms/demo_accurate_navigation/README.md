### Precise Localization Demo

Precise Localization utilizes images to provide precise navigation. Some of its features are:
1. Hold at locations which the vehicles have flown to before
2. Provided with a pre-built model of the surroundings, precisely hold at an appointed location

GPS navigation has limited accuracy that is sometimes insufficient for transportations or surveying, especially when there is a high risk of vehicle collisions. In open space with no disturbance, the standard deviation in the X and Y direction is about 1.0m. The standard deviation is even higher in the Z direction. The performance is worse in high disturbance environment such as in between high-rises and under bridges. 

Computer vision (CV) enabled navigation does not rely on GPS signals. It provides high precision and repeatable localization results. Comparing to RTK, CV-enabled solution does not rely on external links or GPS. CV-enabled navigation can be used indoor and in harsh environments that do not have stable GPS signals. It is affordable and expandable. With pre-built models of the surrounding, drones are able to fly autonomously.

However, CV-enabled navigation is not suitable for an environment that does not have clear features, such as when the camera is facing a large body of still water or is blocked by a single-colour wall.

After going through this demo, you should be able to implement CV-enabled precision localization in your desired use cases. 

---

### 精准定点导航例程

精准定点导航例程使用摄像头的视觉信息进行精准导航。可完成功能有：
	1.重复悬停在之前曾飞行过的航点。
	2.在有模型可依据的场景精准悬停。

在局部来看，GPS导航的精度（无任何干扰的空旷地形时x，y方向标准差 1.0m 左右，z方向更大；有障碍的高楼间，桥下等环境精度更恶劣）有时不足以满足拍摄，运输的需求，尤其是当任务场景中飞行器有碰撞障碍物的风险时。

通过视觉精准定点导航功能，可以不依赖GPS信号，实现相当高精度，高可重复性的定位。
与RTK方案相比，这种方案完全不依赖机外数据链路和GPS电磁环境约束。因此可以在室内和其他GPS信号质量非常恶劣的条件下使用；且成本低廉，可定制性强。如果有场景的三维模型，可以不预先考察而完成指定的定点飞行任务。

然而,这种方法不适合在完全没有特征的场景下使用，如摄像机正对平静的水面或视野完全被纯色的墙填充。


通过运行这一演示，您可以掌握如何使用视觉精准定点模块，并将其部署在自己的应用场景中。
