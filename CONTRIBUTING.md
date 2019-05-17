# Contributing to GAAS
We love your input! We want to make contributing to this project as easy and transparent as possible, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Helping others with their issues/questions

Communication between modules in the software is based on ROS. Each module can be replaced if needed. While replacing modules, the new module must have the same corresponding ROS topic/service pub sub. The project has provided a considerably more stable, but less optimized implementation. We look forward to working with developers around the world to improve the overall project.

## We Use [Github Flow](https://guides.github.com/introduction/flow/index.html), So All Code Changes Happen Through Pull Requests
Pull requests are the best way to propose changes to the codebase (we use [Github Flow](https://guides.github.com/introduction/flow/index.html)). We actively welcome your pull requests:

0. Please ensure you are using the latest version of GAAS with all dependencies installed.
1. Fork the repo and create your branch from `master`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. Issue that pull request!

NOTE: If you have decided to change the API, please update the corresponding README.md with details of changes to the API, including new environment variables, the API itself, relevant file locations and container parameters.

## Any contributions you make will be under the BSD 3-Clause
In short, when you submit code changes, your submissions are understood to be under the same [BSD 3-Clause](https://github.com/ninawrong/GAAS/blob/master/LICENSE) that covers the project. Feel free to contact the maintainers if that's a concern.

## Report bugs using one of our pre-designed [issues template](https://github.com/briandk/transcriptase-atom/issues) 

---------

## 项目结构


软件部分，包括各个飞行器自动驾驶必要的模块。飞控、SLAM、避障、路径规划、高层应用支持、飞行模拟等。

硬件部分，包括廉价可行的旋翼机拼装方案、机上计算机和传感器的配置安装方案。

软件部分各个模块之间的通信使用 ROS 实现。每个模块皆可替换的。进行替换时，只需新模块实现原有对应的 ROS topic/service 的 pub sub 即可。项目中会提供一个较为可靠的实现，但可能不是最优的。我们欢迎开发者提供已有模块的更优实现。

## Pull Request Process

2. 请确保更新对应模块的README，表述清楚**接口的变更**，新引入的全局变量/环境变量等对外部有影响的量，暴露的端口，文件路径和配置项。


### Our Pledge

**We respect all developers who have put in efforts to make autonomous flight possible.**

**We hereby promise that any issue, pull request and suggestions in any form will be treated seriously.**

**Special thanks to all contributors and developers of the project, not only as an ode to your passion towards the project, but also as an ode to what mankind has always yearned.**


**我们尊重所有致力于为飞行器自动驾驶事业做出贡献的开发者。**

**在此保证：你的 issue、pull request 或其他形式给出的建议会被严肃且详细的考虑。**

**感谢一切为此项目做出有益建议和贡献的开发者。不只是为了赞美对项目的热情，而是对人类一直向往的想法所作的礼赞**



