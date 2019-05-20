# Contributing to GAAS
We love your input! We want to make contributing to this project as easy and transparent as possible, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Helping others with their issues/questions
- Implement an optimized algorithms for one of the features.

## Report bugs using one of our pre-designed [issues template](https://github.com/briandk/transcriptase-atom/issues) 
The templates help us to gather informations about what is causing a failure.

## We Use [Github Flow](https://guides.github.com/introduction/flow/index.html), So All Code Changes Happen Through Pull Requests
Pull requests are the best way to propose changes to the codebase (we use [Github Flow](https://guides.github.com/introduction/flow/index.html)). We actively welcome your pull requests:

0. Please ensure you are using the latest version of GAAS with all dependencies installed.
1. Fork the repo and create your branch from `master`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the corresponding README.md.
4. Ensure the test suite passes.
5. Make sure your code lints.
6. Issue that pull request!

NOTE: If you have decided to change the API, please update the corresponding README.md with details of changes to the API, including new environment variables, the API itself, relevant file locations and container parameters. Communication between modules in the software is based on ROS. Each module can be replaced if needed. While replacing modules, the new module must have the same corresponding ROS topic/service pub sub. The project has provided a considerably more stable, but less optimized implementation. 

## Any contributions you make will be under the BSD 3-Clause
In short, when you submit code changes, your submissions are understood to be under the same [BSD 3-Clause](https://github.com/ninawrong/GAAS/blob/master/LICENSE) that covers the project. Feel free to contact the maintainers if that's a concern.

## Pledge
**We respect all developers who have put in efforts to make autonomous flight possible.**

**We hereby promise that any issue, pull request and suggestions in any form will be treated seriously.**

**Special thanks to all contributors and developers of the project, not only as an ode to your passion towards the project, but also as an ode to what mankind has always yearned.**

---------
# 为 GAAS 做贡献
您愿意为 GAAS 做贡献真是太好了。我们希望将这个过程做得尽量简单、透明，你可以通过以下方式帮助我们：

- 提交 Bug
- 跟我们讨论现有代码的状况
- 修补代码
- 提交新的功能
- 帮助他人解决他们遇到的问题
- 为原有功能做性能提升

## 通过我们预先设计好的[issues 模板](https://github.com/briandk/transcriptase-atom/issues)提交 Bug
这个模板可以帮助我们更快速准确地定位问题所在的位置。

## 我们使用 [Github Flow](https://guides.github.com/introduction/flow/index.html)对 GAAS 进行迭代, 因为所有代码更新都经过 Pull Request 流程
Pull requests 是管理代码变动最好的方法 (我们使用[Github Flow](https://guides.github.com/introduction/flow/index.html))。我们随时欢迎给我们提 PR：

0. 确保您在使用最新版本的 GAAS，并已安装好所有依赖。
1. Fork 代码并从 master 中创建一个 branch。
2. 如果你添加了需要测试的代码，请提供测试脚本。
3. 如果你更改了 API，请对相应的 README.md 做更新。
4. 确保代码在您的机器上测试通过。
5. 保持代码整洁。
6. 提交 Pull Request

NOTE：请确保更新对应模块的README，表述清楚**接口的变更**，新引入的全局变量/环境变量等对外部有影响的量，暴露的端口，文件路径和配置项。软件部分各个模块之间的通信使用 ROS 实现。每个模块皆可替换的。进行替换时，只需新模块实现原有对应的 ROS topic/service 的 pub sub 即可。项目中会提供一个较为可靠的实现，但可能不是最优的。我们欢迎开发者提供已有模块的更优实现。

## 您做的所有贡献将受到 BSD 3-Clause 协议保护
当您在提交代码的时候，您的代码将默认受到 [BSD 3-Clause](https://github.com/ninawrong/GAAS/blob/master/LICENSE)协议保护。BSD 3-Clause 也是 GAAS 项目整个项目的开源协议。如果您有任何问题，请联系 GAAS 管理员。

## 项目宣言

**我们尊重所有致力于为飞行器自动驾驶事业做出贡献的开发者。**

**在此保证：你的 issue、pull request 或其他形式给出的建议会被严肃且详细的考虑。**

**感谢一切为此项目做出有益建议和贡献的开发者。不只是为了赞美对项目的热情，而是对人类一直向往的想法所作的礼赞。**



