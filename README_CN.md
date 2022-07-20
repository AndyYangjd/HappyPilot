# JoyPilot

免责声明：本项目仅是一个joy，请不要用于实际的现实场景中。

----

## 一、起源

本项目起源于今晚一个不成熟的快乐想法。去年曾经花了一小段时间学习了Autoware.Auto项目，基本了解了如何基本Ros2建立一个大的ADAS软件工程框架。
不幸的是，最后在公司内部由于一些原因不能继续开展下去。今年由于晚上时间相对宽松，便计划在工作之外时间继续Autoware.Auto的学习。

本工程主要用于用于个人的学习研究中，来源于裁剪Autoware.Auto工程，裁减原因有如下：

1. 依赖项和新工具过多：很多人都迷失在ade等环境的配置上面。不想造轮子的工程师不是好工程师，Autoware.Auto在这方面表现的很出色@-@
2. 工程想方方面面包含，对新人不友好。例如在tools目录下有很多launch文件，一些包的命名也很有迷惑性
3. LgSVL仿真器官方停止支持。从这个方面考虑，当前的最佳开源仿真器是carla，因此计划在此工程中将仿真迁移到 carla 中

## 二、相似点

本工程的文件夹布局，文件、函数等命名网格将与Autoware.Auto保持一致。

## 三、工程相关情况

- GitHub：https://github.com/AndyYangjd/JoyPilot.git
- 分支说明：
  * main：未来主要的释放分支
  * dev：主要的开发合并分支
  * andy：我个人的开发分支
- 参与：如果有想参与的小伙伴，可以私信我沟通如何分配Autoware.Auto工程裁剪。

## 四、工程结构
工程从整体上分为三个层级：Src(根目录) ---> Module(模块) ---> Package(包)

  * Src（根目录）：包含工程所有的源代码
  * Module（模块）：模块是自动驾驶中感知、控制等大模块的抽象
  * Package（包）：包是每个模块下任务的分解


## 五、命名风格

### 5.1头文件开头注释：
``` c++
#ifndef <Moduel-Name>__<Package-Name>__<File-Name>_
#define <Moduel-Name>__<Package-Name>__<File-Name>_

/*
code here
*/

#endif  // <Moduel-Name>__<Package-Name>__<File-Name>_
```

### 5.2 命名空间
```c++
namespace <Project-Name> { // Project-Name，统一为  joypilot
namespace <Module-Name> { // Module-Name
namespace <Package-Name> { // Package-Name
/*
code here
*/    
} // namespace <Project-Name> 
} // namespace <Module-Name> 
} // namespace <Package-Name> 
```


