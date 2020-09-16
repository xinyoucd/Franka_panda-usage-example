# Franka_panda-usage-example

## 本手册使用说明

+ `本手册并不是一个完整的官方说明手册，而是一个`***经验使用手册***`;`
+ `本手册的目的时让开发者` ***快速的上手*** `使用franka_panda机械臂，并对相关的功能进行了一定的介绍；`
+ `本手册不会像官方的手册那样` **非常严谨** `，主要的目的让大家快速的上手这款机械臂；`
+ `后续的细节请参照` ***官方的文档*** `进行学习；`

## Franka_panda 图片欣赏

<img src="/image_view/Franka_Panda_179.png" width="300" /><img src="/image_view/Franka_Panda_176.png" width="300" /><img src="/image_view/Franka_Panda_175.png" width="300" />

<img src="/image_view/Franka_Panda_145.png" width="300" /><img src="/image_view/Franka_Panda_113.png" width="300" /><img src="/image_view/Franka_Panda_112.png" width="300" />

## Franka_panda 机械臂欣赏

+ 展示一：

<center>

<img src="/image_view/panda-power-tool.eebfa39.gif" width="1000" />

</center>

+ 展示二：

<center>

<img src="/image_view/intro.b081286.gif" width="1000" />

</center>

## 开箱说明

<center>

<img src="/image_view/panda_view.jpg" width="700" />

</center>

+ [x] `Franka_panda 机械臂一台`
+ [x] `控制柜一台`
+ [x] `夹持器一个`
+ [x] `急停按钮`
+ [x] `运动模式控制按钮`
+ [x] `手柄控制器`
+ [x] `其他配件`
+ **选购产品**
  + [ ] **FCI控制插件** ***===>*** `ros通讯插件;`
  + [ ] `官方的`**App**`控制插件` ***===>*** `使用官方的自己的运动控制插件;`

## 声明

+ `本产品的相关配置已经全部配置完成`
+ `不需要重置`

## 机械臂状态

+ **黄色灯亮**
  + ***锁定状态***
+ **蓝色灯亮**
  + ***准备运动模式***
+ **粉红色灯亮**
  + ***报警，不会急停***
+ **红色灯亮**
  + ***急停***
+ **白色灯亮**
  + ***没有使能***

## Franka_panda 相关信息

+ **Franka_panda** `控制柜IP(`**静态ip**`)`
  + ***172.16.0.2***
+ **主控机的IP**`(静态ip)`
  + ***172.16.0.1***
+ **注意**`:`
  + `在`**服务模式**`下，地址为`**自动获取**`的`**IP**`，不是`**静态IP**`;`

## Franka_panda官方资源

+ **官方网站**
  + <https://www.franka.de/technology>
+ **官方教程**
  + <https://frankaemika.github.io/docs>
+ ***libfranka*** **下载地址:**
  + <https://github.com/frankaemika/libfranka>
+ ***libfranka*** **API手册**
  + <https://frankaemika.github.io/libfranka/>
+ ***frank_panda*** **ROS通讯接口**
  + <https://github.com/frankaemika/franka_ros>
+ ***panda_ros*** **wiki教程**
  + <https://wiki.ros.org/franka_ros>

## 准备工作

+ **相关工具准备**
  + [x] `一台` **Franka Panda**`机器人;`
  + [x] `装有`**Linux**`的电脑一台;`
  + [x] `一根质量较好，较长的`**网线**`;`
+ **基础知识**
  + [x] **机器人学**`的相关知识`;
  + [x] **C++编程**;
  + [x] **Linux操作系统** `的相关知识`;
  + [x] **ROS系统** `的相关知识`;
+ ***linux*** **系统版本**
  + ***Ubuntu 16.04*** (**Xenial Xerus**)
+ ***ROS*** **版本**
  + ***rosdistro:*** **kinetic**
  + ***rosversion:*** **1.12.14**
  
## 使用注意事项

+ `机器人使用`**注意事项**：
  + `由于panda机械内部的机械结构构造，在使用时候注意以下几点:`
    + `panda机械臂使用时,请将机械臂的`**底部固定牢固**`。如果，底部固定的不够牢固，可能导致机械臂发生急停；`
    + `使用此机械臂时，应避免`**关节扭矩过失**;
    + `规划机械臂的运动轨迹时，尽量避免` **关节扭矩过失**`的`**运动规划**;
  + **急停的处理：**
    + `按下急停之后，请立即`**关闭控制柜的电源**`；`
    + `此时，打开开关`**不能启动机械臂**`为正常现象。请关闭电源，`**等待1~2分钟**`后重启;`
+ `将机器人组装完成之后，先使用`**Franka_panda机器人**`自带的`**Desk界面**`测试机器人的相关功能是否正常;使用Desk界面的步骤如下:`
  + `首先,用`**以太网线**`连接自己的电脑和机器人。`
  + **注意:**
    + **Franka机器人**`有两个`**以太网口**，`一个在机器人`**底座**`上，一个在`**控制柜**`上。二者是不同的，但是都可以使用。`
+ `两个端口的`**区别**：
  + `如果连接到`**机器人底座** `，此时机器人是` **server模式**`，可以通过`***robot.franka.de*** `访问desk。`
  + `如果连接到`**控制柜**，`此时机器人是`**client模式**`，只能通过ip地址访问desk。`
+ **备注：**
  + `如果机器人使用`**franka**`自带的`**app**`,连接时；`
    + `可以同时使用两种模式，`**server模式**`和`**client模式;**
  + `如果使用`**ROS**`进行控制时：`
    + `只能使用`**client模式;**
  + **Desk**`的用法十分简单，此时不做介绍。`
  
## 相关的软件安装

+ `此手册提供的软件安装的系统为`**Linux操作系统**`，`**Franka-panda机械臂**`进行编程控制时，一般有两种模式，即使用`**ROS**`或者直接调用`**API**`。本教程提供的两种方式进行控制;`
+ **安装ROS**
  + `安装教程点击这里，`**:point_right:** [***教程***](http://wiki.ros.org/kinetic/Installation)
  + **:smirk: :smirk: :smirk:**
+ **libfranka** `源码安装编译`
  + `安装最新版本的`**libfranka**`，必须采用`**源码编译**`，首先，删除之前安装的`**libfranka**`和`**franka_ros**`以防冲突。`
  
  ```bash
  sudo apt remove "*libfranka*"
  ```

  + `删除完成后，首先安装依赖库：`
  
  ```bash
  sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
  ```

  + `首先将`***Source code***`文件夹里面的`***libfranka***`取出来放到`***home***`文件夹下面`
  + `具体的编译源码的指令如下:`
  
  ```bash
  # 创建源码构建空间
  mkdir build

  # 进入到源码构建的空间
  cd build

  # 构建源码
  cmake -DCMAKE_BUILD_TYPE=Release ..
  
  # 编译源码
  cmake --build .
  ```

+ **编译、构建** ***franka_ros*** `通讯接口`
  + `功能包安装`
  
  ```bash
  sudo apt-get install ros-kinetic-franka-ros
  ```

  + **构建工作空间**
    + `构建工作空间指南,点击这里` **:point_right:** [工作空间构建](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
    + `操作如下:`
  
    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspeace
    cd ..
    catkin_make
    ```

    + `将提供的源码放到工作空间里面;`
    + `编译源码的步骤如下:`

    ```bash
    #进入到工作空间
    catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build

    # 刷星ros工作空间脚本
    source devel/setup.sh
    ```

+ `编译`**real-time内核**
  + `本节是以4.14.12内核为基础，不同的版本需要的内核不同，此处主要是演示内核的编译过程;`
  + `使用`**Franka_panda机械臂时,**`需要`**实时内核。**`这种方式和绝大多数使用`**ROS**`控制的机械臂有很多的不同。`**linux**`的`**实时内核**`的构建方法有多种`。`本教程使用的是`**Franka_panda官方**`提供的`*实时内核*`的编译方法:`
  + `查看当前系统的`**Linux内核*****版本***`，终端输入：`
  
  ```bash
  uname -r
  ```
  
  + **终端**`会提示当前`**Linux内核的版本**`，在`**ubuntu16.04.6**`版本的系统在终端里面显示的内核版本为--->`**4.15.0-generic**`。终端界面显示的版本不是`**实时内核**`，需要自行安装编译`**实时内核**`。选择`**实时内核**`的版本没有什么推荐，原则上使用一个与自己版本最接近的内核就好。如果担心`**内核奔溃**`，可以安装多个`**内核**`并在`**高级启动**`时自行切换。`
  + `内核的安装编译方法可参考`**Franka FCI**`手册，但是该方法在网络连接不畅时极其慢，不推荐使用。`
  + **注意：**
    + `选择`**实时内核**`版本时，不必选择与自带内核号一致的版本。也可以安装对应的其他的`**generic内核**。`有时候甚至可能出现在某一版本内核下编译出错，换一个内核却可以的情况。`
    + `编译过程非常费时，完成后需要调整`**默认的内核**`启动系统才能自行进入`**实时内核**。
    + `有些时候可能会`**无法进入**`所选`**内核**`，系统提示“`**vmlinuz-… invalid signiture**`”，解决方案是在`**BIOS**`里面关闭`**Secure boot**`功能。`
    + `系统启动后别忘了用`**uname -r**`或者`**uname -a**`检查内核是否已经正确切换到`**实时内核**`。`
    + **内核**`是安装在系统的`**boot分区**`下的，因此务必保证该分区空间充足，如何希望保存两个内核（`**generic** `和` **rt 内核**`）的话，建议`**boot分区**`大于`**4G**`，以便后续内核更新不会出现问题。`
  + ***指令如下：***
  
  ```bash
  # 下载实时内核 && 相关的配置文件
  curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.xz
  curl -SLO https://www.kernel.org/pub/linux/kernel/v4.x/linux-4.14.12.tar.sign
  curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
  curl -SLO https://www.kernel.org/pub/linux/kernel/projects/rt/4.14/older/patch-4.14.12-rt10.patch.xz
  ```

  + **注意**：
    + `此处下载极慢，最好使用一些加密协议`

  + **解压**`下载的相关文件`

  ```bash
  xz -d linux-4.14.12.tar.xz
  xz -d patch-4.14.12-rt10.patch.xz
  ```

  + `对下载的`**实时内核包**`进行`**数据校验**`;`

  ```bash
  gpg2 --verify linux-4.14.12.tar.sign
  ```

  + `显示`**校验的信息**`如下:`

  ```bash
  gpg: assuming signed data in 'linux-4.14.12.tar'
  gpg: Signature made Fr 05 Jan 2018 06:49:11 PST using RSA key ID 6092693E
  gpg: Can't check signature: No public key
  ```

  ```bash
  # 0x6092693E这个秘钥时根据gpg2 --verify linux-4.14.12.tar.sign得到的
  gpg2  --keyserver hkp://keys.gnupg.net --recv-keys 0x6092693E

  gpg2 --verify patch-4.14.12-rt10.patch.sign

  # 0x2872E4CC验证秘钥是根据gpg2 --verify patch-4.14.12-rt10.patch.sign得到的
  gpg2 --keyserver hkp://keys.gnupg.net --recv-keys 0x2872E4CC
  ```

  + `上面的`**步骤**`是为了校验实时内核的`**完整性**`;`
  + `如果您对自己的网络比较自信可以忽略;`
  + :joy::joy::joy:
  + **解压源码包**`，准备开始`**编译源码包**`，指令如下:`

  ```bash
  #解压源码包
  tar xf linux-4.14.12.tar

  #进入到源码包
  cd linux-4.14.12

  #添加源码包的配置
  patch -p1 < ../patch-4.14.12-rt10.patch

  #编译源码包
  make oldconfig
  ```

  + `上面的`**指令**`输完之后，`**终端里面**`就会显示下面的界面；`

  ```bash
  Preemption Model
    1. No Forced Preemption (Server) (PREEMPT_NONE)
    2. Voluntary Kernel Preemption (Desktop) (PREEMPT_VOLUNTARY)
    3. Preemptible Kernel (Low-Latency Desktop) (PREEMPT__LL) (NEW)
    4. Preemptible Kernel (Basic RT) (PREEMPT_RTB) (NEW)
    > 5. Fully Preemptible Kernel (RT) (PREEMPT_RT_FULL) (NEW)
  ```

  + `当源码`**构建**`完成之后，输入下面的`**指令**`开始`**编译内核**`；这个过程取决于自己电脑的`**cpu**`的性能以及内核数目,后面`**编译内核的参数**`可以根据自己电脑的情况进行修改;`

  ```bash
  fakeroot make -j4 deb-pkg
  ```

  + `编译完成之后，输入下面的指令，`**安装**`编译好的实时内核；`

  ```bash
  sudo dpkg -i ../linux-headers-4.14.12-rt10_*.deb ../linux-image-4.14.12-rt10_*.deb
  ```

  + `输入下面的指令，调整`**CPU**`的工作模式;`

  ```bash
  sudo addgroup realtime

  sudo usermod -a -G realtime $(whoami)

  sudo gedit /etc/security/limits.conf
  ```

  + `向系统的`**添加cpu相关的配置**

  ```bash
  @realtime soft rtprio 99
  @realtime soft priority 99
  @realtime soft memlock 102400
  @realtime hard rtprio 99
  @realtime hard priority 99
  @realtime hard memlock 102400
  ```
  
  + `禁用`**CPU节能策略**
    + `为了`**安全性**`和`**性能**`起见，建议配置完成后关闭系统的CPU频率调整功能。`
    + `首先安装工具`**cpufrquentiles**`：`
  
    ```bash
    sudo apt install cpufrequtils
    ```

    + `完成后，运行`**cpufreq-info**`查看当前`**CPU状态**`，当前“`**governor**`”属性可能是“`**powersave**`”模式，`**修改**`为“`**performance**`”模式。`
  
    ```bash
    cpufreq-info
    ```

    + `修改方法有多种，可以采用`**indicator-cpufreq工具**`在`**UI界面**`手动修改。也可通过指令修改，运行如下指令：`

    ```bash
    sudo systemctl disable ondemand
    sudo systemctl enable cpufrequtils
    sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
    sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
    ```

## 如何使用源码

+ `首先，安装提供编译的源码；`
+ **API** ***demo说明***
  + ***communication_test***
    + **通信测试**`;`
    + `使用方法`

    ```bash
    ./examples/communication_test "172.16.0.2"
    ```

  + ***cartesian_impedance_control***
    + `无惯性整形的简单`**笛卡尔阻抗控制器**`;`
    + `使用方法`

    ```bash
    ./examples/cartesian_impedance_control "172.16.0.2"
    ```

  + ***echo_robot_state***
    + `连续`**读取机器人状态**`;`
    + `使用方法`

    ```bash
    ./examples/echo_robot_state "172.16.0.2"
    ```

  + **force_control**
    + **PI力控制器**`，可在`**Z轴**`上绘制对应的重力;`
    + `使用方法`

    ```bash
    ./examplesforce_control "172.16.0.2"
    ```

  + ***generate_cartesian_pose_motion***
    + `如何生成`**笛卡尔运动**`;`
    + `使用方法`

    ```bash
    ./examples/generate_cartesian_pose_motion "172.16.0.2"
    ```
  
  + ***generate_cartesian_velocity_motion***
    + `如何生成`**笛卡尔速度运动**`;`
    + `使用方法`

    ```bash
    ./examples/generate_cartesian_velocity_motion "172.16.0.2"
    ```
  
  + ***generate_consecutive_motions***
    + `错误恢复的`**连续运动**`;`
    + `使用方法`

    ```bash
    ./examples/generate_consecutive_motions "172.16.0.2"
    ```
  
  + ***generate_elbow_motion***
    + `移动机器人`**肘部**`;`
    + `使用方法`

    ```bash
    ./examples/generate_elbow_motion "172.16.0.2"
    ```

  + ***generate_joint_position_motion***
    + `如何产生`**关节位置运动**`;`
    + `使用方法`

    ```bash
    ./examples/generate_joint_position_motion "172.16.0.2"
    ```

  + ***generate_joint_velocity_motion***
    + `如何产生`**关节速度运动**`;`
    + `使用方法`

    ```bash
    ./examples/generate_joint_velocity_motion "172.16.0.2"
    ```

  + ***grasp_object***
    + `控制`**FRANKA夹持器**`;`
    + `使用方法`

    ```bash
    ./examples/grasp_object "172.16.0.2"
    ```

  + ***joint_impedance_contro***
    + `以形状执行`**笛卡尔运动**`的`**关节阻抗类型控件**`,`**画圆**`;`
    + `使用方法`

    ```bash
    ./examples/joint_impedance_contro "172.16.0.2"
    ```

  + ***joint_point_to_point_motion***
    + `命令`**关节位置**`将机器人移动到`**目标位置**`;`
    + `使用方法`

    ```bash
    ./examples/joint_point_to_point_motion "172.16.0.2"
    ```
  
  + ***motion_with_control***
    + `使用`**电机控制**`和`**扭矩控制**;
    + `使用方法`

    ```bash
    ./examples/motion_with_control "172.16.0.2"
    ```

  + ***print_joint_poses***
    + `每个关节相对于`**基础框架**`的`**矩阵**`;`
    + `使用方法`

    ```bash
    ./examples/print_joint_poses "172.16.0.2"
    ```

+ **Frank_ros**
  + `启动`**move_it**`运动规划`
  
  ```bash
  roslaunch panda_moveit_config panda_control_moveit_rviz.launch
  #不用添加机械臂的IP地址
  #机械臂的IP地址已在launch文件中写好
  ```

  + `等待所有的`**规划组件**`启动完成之后，在`**rviz**`里面拖动末端，进行机械臂的`**运动规划**`;`
  + `使用时，`**Franka_panda**`的指示灯一定为`**蓝色**

## 关于Franka_panda生态升级

+ **Franka_panda机械臂**`的测试环境为`**Ubuntu16.04**`，但是考虑到`**Ubuntu**`有效的系统支持有效期为`**5年**`，近期将`**Ubuntu18.04**`的测试结果进行说明:`
  + [x] **linux 系统升级**`到`**Ubuntu18.04;**
  + [x] **ROS**`版本升级到`**Melodic**`;`
  + [x] **libfrank** `官方库编译测试通过;`
  + [x] **franka_panda** **ros通讯借口**`测试完成;`
  + [x] **real-time kernel**`编译通过，并提供相应的内核；`
  + [x] `相关的`**源码整合包**`也在调试当中;`

## 最后的声明

+ `此文档的源码是`**测试后的源码**`,只对购买我们公司产品的`**客户提供**`，`**其他人员**`不在我们的考虑范围之内;`
+ `由于，不同的客户购买的`**批次不同**`，我们所`**提供的文件**`会有所不同;`
+ `如果，想用最新的`**libfranka库**`,请将`**panda的固件刷**`到最新;`
