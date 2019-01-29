CONTRIBUTING for GAAS

# Roadmap

我们的目标是加速飞行器全自动驾驶的广泛应用。这里的飞行器包括现在被广泛使用的无人机，也包括未来的小型载人飞行器。

这个项目的目的在于提供一个快速可上手的，稳定的飞行器自动驾驶解决方案。这套方案分为软件和硬件两个部分。


软件部分，包括各个飞行器自动驾驶必要的模块。飞控、SLAM、避障、路径规划、高层应用支持、飞行模拟等。

硬件部分，包括廉价可行的旋翼机拼装方案、机上计算机和传感器的配置安装方案。



软件部分各个模块之间的通信使用 ROS 实现。每个模块皆可替换的。进行替换时，只需新模块实现原有对应的 ROS topic/service 的 pub sub 即可。项目中会提供一个较为可靠的实现，但可能不是最优的。我们欢迎开发者提供已有模块的更优实现。




# Contributing


向本项目提交代码前,请先通过 issue 或邮件向我们提出和讨论你想做的改变。

When contributing to this repository, please first discuss the change you wish to make via issue, email, or any other method with the owners of this repository before making a change. 


请注意：本项目对代码规范有一定要求，请遵守这些要求。

Please note we have a code of conduct, please follow it in all your interactions with the project.



## Pull Request Process

1.请确保提交的部分代码的依赖项已经完善处理。

1. Ensure any install or build dependencies are removed before the end of the layer when doing a build.


2.请确保更新对应模块的README，表述清楚**接口的变更**，新引入的全局变量/环境变量等对外部有影响的量，暴露的端口，文件路径和配置项。

2. Update the README.md with details of changes to the interface, this includes new environment variables, exposed ports, useful file locations and container parameters.


3.请确保更新依赖本次变更的示例文件版本号，并在 README 中指出版本变更。

3. Increase the version numbers in any examples files and the README.md to the new version that this Pull Request would represent. 


4.如果你没有直接合并 pull request 的权限,你可能需要联系我们帮你合并。

4. If you do not have permission to merge your pull request directly, you may request the reviewer to merge it for you.


### Our Pledge

**我们尊重所有致力于为飞行器自动驾驶事业做出贡献的开发者。**

**在此保证：你的 issue、pull request 或其他形式给出的建议会被严肃且详细的考虑。**

**感谢一切为此项目做出有益建议和贡献的开发者。不只是为了赞美对项目的热情，而是对人类一直向往的想法所作的礼赞**



