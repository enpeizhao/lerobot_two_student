[toc]

> 下面的流程、代码、工具等只适用于双臂遥操作系统，单臂遥操作系统请查看上一篇。

## 一、遥操作系统校准

### 1.1 环境配置

* Ubuntu电脑：我的版本是20.04.6 LTS，其他版本，尤其是更高版本，可能有些驱动装不了

* 使用Anaconda创建虚拟环境：`conda create -y -n lerobot python=3.10`

* 激活环境：`conda activate lerobot`

* 下载修改后的`lerobot`代码：`git clone https://github.com/enpeizhao/lerobot_two_student.git`

  > 不要使用[lerobot 官方代码](https://github.com/huggingface/lerobot?tab=readme-ov-file)，因为有代码改动（官方的代码不支持我的机械臂）

* 解压，进入目录`cd lerobot`

* 安装`lerobot`，使用：`pip install -e . -i https://mirrors.huaweicloud.com/repository/pypi/simple `

  > 注意：如果之前在环境中装过单臂的lerobot，这里需要覆盖安装。

* 安装飞特舵机支持，使用：`pip install 'lerobot[feetech]'`

* 我的环境依赖参考在根目录的`lerobot_env_two.txt`



### 1.2 中位校准遥操主臂

#### 1.2.1 右侧遥操主臂

> 右侧遥操主臂的配置过程，与上一篇单臂遥操一模一样

* 将右侧遥操主臂的电源和数据线插上，数据线另一头连接电脑

* 运行指令：`ls /dev/ttyACM*  `，检查驱动板在Ubuntu下的端口，比如我的是：`/dev/ttyACM0`

* 运行：`python -m lerobot.set_middle --port=/dev/ttyACM0`，进入校准程序，应该会输出这样的界面：

  ```bash
  INFO 2025-09-10 17:19:06 t_middle.py:117 {'motor_range': (1, 7), 'port': '/dev/ttyACM0'}
  Connected to Feetech motors on port /dev/ttyACM0
  已解锁所有电机（扭矩禁用）
  开始持续监控位置。电机范围: 1-7。按 Ctrl+C 停止。
  您可以在监控时手动移动机械臂。
  按 'r' 重置中位位置。
  按 'l' 切换电机锁定状态（启用/禁用扭矩）。
  原始位置: 电机 1: [2911], 电机 2: [1678], 电机 3: [3691], 电机 4: [2200], 电机 5: [1357], 电机 6: [1903], 电机 7: [2062] 
  ```

* 转动遥操主臂各个关节，你会看到电机位置数值变化

  > 飞特总线舵机编码器的分辨率是4096，即你会看到位置在（0~4096）范围内

* 切换英文输入法，按`r` ，会提示：`请手动移动机械臂到新的中位位置，然后按回车... `

  > 如果没有响应，可以先按一下回车键

* 将右遥操主臂各个关节转到它的中间位置（编码器2048的位置），大概如下：

  * 不必要求特别精准，大概在中位就行了
  * 需要注意夹爪支架的方向，不然校准后不顺手
  * 需要注意夹爪手指环的要尽量在中位，不然可能遥操作的执行的主臂夹爪抓不紧
  * 当然，校准不是一次性的，如果不满意，随时可以重新校准

  | 侧视图                                                       | 后视图                                                       | 正视图                                                       |
  | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509101724094.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509101724162.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509101727850.png?x-oss-process=style/resize) |

* 保持遥操主臂静止，按回车键，你会看到输出的电机角度全部校准为了2048
* 退出中位校准程序，保持遥操主臂通电、数据线连接电脑。



#### 1.2.2 左侧遥操主臂

* 类似的，将左侧遥操主臂的电源和数据线插上，数据线另一头连接电脑

* 我们运行指令：`ls /dev/ttyACM*  `，检查驱动板在Ubuntu下的端口，这时应该有两个端口，比如我的：

  ```
  /dev/ttyACM0  /dev/ttyACM1
  ```

* `/dev/ttyACM0`是右侧的端口，那么另一个就是左侧的端口。运行：`python -m lerobot.set_middle --port=/dev/ttyACM1`，进入校准程序

* 将左侧遥操主臂各个关节转到它的中间位置（编码器2048的位置），如下：

  * 如下图“左手——正视图” 前6个关节与右手一致
  * 区别是左手夹爪与右手夹爪差180度（这样更方便握住）。具体对比，请查看下图

  | 左手——正视图                                                 | 左手夹爪——正视图                                             | 右手夹爪——正视图                                             |
  | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509160946651.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509160944808.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509160944171.png?x-oss-process=style/resize) |

* 保持遥操主臂静止，按回车键，你会看到输出的电机角度全部校准为了2048
* 退出中位校准程序，保持遥操主臂通电、数据线连接电脑。



### 1.3 机械臂遥操作测试

> <font color="red">操作时，注意安全！！！</font>
>
> <font color="red">操作时，注意安全！！！</font>
>
> <font color="red">操作时，注意安全！！！</font>
>
> <font color="red">第一次最好找一个助手协助，如果执行的从臂异常运动，可以及时断电（注意用手托住关节）</font>

* 摆放机械臂：

  * 机械臂先别装相机和夹爪（忽略下图）
  * 两台执行从臂相对摆放（插线槽相互远离，方便走线），通电、数据线连接电脑
  * 两台遥操主臂同样相对摆放（插线槽相互远离，方便走线），通电、数据线连接电脑

  | 俯拍                                                         | 正视                                                         | 遥操作                                                       |
  | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161102176.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161103262.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161017527.png?x-oss-process=style/resize) |

  

* 修改左右2台Episode1机械臂（注意，不是遥操主臂）六个驱动板的参数，将`Response`改为`None`，如果不会修改，请[查看这里](https://enpeicv.com/forum.php?mod=viewthread&tid=1541&extra=page%3D1)

* 都插上夹爪控制盒，夹爪先不用装在机械臂末端（还没有安装相机）

* 下载最新版上位机（V0.9.8以上）

* 左侧Episode1机械臂上电

  * 打开上位机，选择对应的USB_ID，设置一个端口
  * 启用服务器，进行归零、回到默认位置

* 右侧Episode1机械臂上电

  * 再打开一个上位机窗口，选择对应的USB_ID，设置一个端口
  * 进行归零、回到默认位置

  > 注意：两个上位机的USB_ID、端口不可能一致，否则另一台无法启动，比如我配置：
  >
  > ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509160959240.png?x-oss-process=style/wp)

* 2个上位机关闭”启用日志“、”启用状态刷新“ 复选框

* 运行下面代码，机械臂会运行到准备位置

  ```bash
  python -m lerobot.episode_default_position --ip_left=localhost --port_left=12346 --ip_right=localhost --port_right=12345
  ```

  ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161037899.png?x-oss-process=style/resize)

* 双手握住各自遥操主臂，初始位置与从臂接近

* 运行指令：

  > <font color="red">操作时，注意安全！！！</font>

  ```bash
  python -m lerobot.teleoperate \
      --robot.ip_address_left="localhost" \
      --robot.port_left=12346 \
      --robot.ip_address_right="localhost" \
      --robot.port_right=12345 \
      --robot.type=enpei_follower \
      --robot.id=enpei_follower \
      --robot.cameras="{ }" \
      --teleop.type=enpei_leader \
      --teleop.port_left=/dev/ttyACM1 \
      --teleop.port_right=/dev/ttyACM0 \
      --teleop.id=enpei_leader \
      --fps=30\
      --display_data=false \
      --enpei_speed_mode=record
  ```

  robot表示从臂执行臂相关参数，telep表示遥操主臂相关参数，在测试时，你只需要修改下列标 “✅ ”的项：

  | 参数                                 | 是否需要修改 | 解释                                                 |
  | ------------------------------------ | ------------ | ---------------------------------------------------- |
  | `robot.ip_address_left="localhost"`  | ✅ 是         | 左侧从臂上位机API的IP地址                            |
  | `robot.port_left=12346`              | ✅ 是         | 左侧从臂上位机API的端口                              |
  | `robot.ip_address_right="localhost"` | ✅ 是         | 右侧从臂上位机API的IP地址                            |
  | `robot.port_right=12345`             | ✅ 是         | 右侧从臂上位机API的端口                              |
  | `robot.type=enpei_follower`          | 否           | 从臂类别，方便框架识别                               |
  | `robot.id=enpei_follower`            | 否           | 从臂ID，方便框架识别                                 |
  | `robot.cameras="{ }"`                | 否           | 相机参数，测试时暂不需要填写，实际采集数据的时候需要 |
  | `teleop.type=enpei_leader`           | 否           | 主臂类别，方便框架识别                               |
  | `teleop.port_left=/dev/ttyACM1`      | ✅  是        | 左侧主臂驱动板端口                                   |
  | `teleop.port_right=/dev/ttyACM0`     | ✅  是        | 右侧主臂驱动板端口                                   |
  | `teleop.id=enpei_leader`             | 否           | 主臂ID，方便框架识别                                 |
  | `fps=30`                             | 否           | 控制频率，范围0~100，越大，遥操作响应越快            |
  | `display_data=false`                 | 否           | `rerun.io`可视化                                     |
  | `enpei_speed_mode=record`            | 否           | 速度模式                                             |

* 一切正常的话，程序启动后，便可以用遥操主臂操作从臂了，请检查：

  * 分别转动左侧主臂前6个关节，看看从臂能否跟着转动

  * 看看左侧夹爪能否控制

  * 再检查右侧

  * 终端应该有类似如下输出：

    <img src="https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509152226455.png?x-oss-process=style/resize" style="zoom: 67%;" />

* 自己遥操作熟练一下，如果需要退出，在终端按`Ctrl+C`退出程序

* 高速模式：

  > <font color="red">操作时，注意安全！！！</font>
  >
  > <font color="red">操作时，注意安全！！！</font>
  >
  > <font color="red">操作时，注意安全！！！</font>

  ```bash
  python -m lerobot.teleoperate \
      --robot.ip_address_left="localhost" \
      --robot.port_left=12346 \
      --robot.ip_address_right="localhost" \
      --robot.port_right=12345 \
      --robot.type=enpei_follower \
      --robot.id=enpei_follower \
      --robot.cameras="{ }" \
      --teleop.type=enpei_leader \
      --teleop.port_left=/dev/ttyACM1 \
      --teleop.port_right=/dev/ttyACM0 \
      --teleop.id=enpei_leader \
      --fps=100\
      --display_data=false \
      --enpei_speed_mode=teleop
  ```



## 二、安装测试相机

* 再次运行指令，让从臂运行到遥操默认位置，方便安装相机和夹爪：

  ```bash
  python -m lerobot.episode_default_position --ip_left=localhost --port_left=12346 --ip_right=localhost --port_right=12345
  ```

* 先安装腕部相机，再安装夹爪

  * 相机尽量在正前方，这样可以拍到夹爪
  * 可以用魔术贴扎带将线固定好

  | 位置1                                                        | 位置2                                                        |
  | ------------------------------------------------------------ | ------------------------------------------------------------ |
  | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509111728325.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509111728451.png?x-oss-process=style/resize) |

* 将固定位相机也摆好：

  <img src="https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161103262.png?x-oss-process=style/resize" style="zoom: 67%;" />

* 3个相机的USB线插入电脑USB口，最好是USB3口

  * 如果线不够长，需要自购一根USB3延长线
  * 如果电脑没有多余的USB3接口，需要自购USB3扩展坞

* 运行：`python -m lerobot.find_cameras opencv `，应该输出类似信息：

  ```bash
  --- Detected Cameras ---
  Camera #0:
    Name: OpenCV Camera @ /dev/video0
    Type: OpenCV
    Id: /dev/video0
    Backend api: V4L2
    Default stream profile:
      Format: 0.0
      Width: 640
      Height: 480
      Fps: 30.0
  --------------------
  Camera #1:
    Name: OpenCV Camera @ /dev/video2
    Type: OpenCV
    Id: /dev/video2
    Backend api: V4L2
    Default stream profile:
      Format: 0.0
      Width: 640
      Height: 480
      Fps: 30.0
  --------------------
  Camera #2:
    Name: OpenCV Camera @ /dev/video4
    Type: OpenCV
    Id: /dev/video4
    Backend api: V4L2
    Default stream profile:
      Format: 0.0
      Width: 640
      Height: 480
      Fps: 30.0
  --------------------
  
  Finalizing image saving...
  Image capture finished. Images saved to outputs/captured_images
  ```
  
  * 必须有3个相机（分割线分割），如果数量不对，请检查连线
  
  * FPS必须都要达到30
  
  * 确定ID，去`outputs/captured_images`下保存的图片查看
  
    * 结合文件名和拍摄内容，可以看到各自相机的索引：
      * 左侧相机：4
      * 右侧相机：0
      * 俯拍：2
    
    * 腕部相机必须要能看到柔性夹爪的手指（下方蓝色、红色手指），否则需要调整你的相机位置
    
    | opencv__dev_video0.png                                       | opencv__dev_video2.png                                       | opencv__dev_video4.png                                       |
    | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
    | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161107512.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161108415.png?x-oss-process=style/resize) | ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161108724.png?x-oss-process=style/resize) |



* 结合相机，再次遥操作：

  * 左侧相机：`handeye_left`，4
  * 右侧相机：`handeye_right`，0
  * 俯拍：`fixed`，2
  * `display_data=true ` 表示打开`rerun.io`可视化（要在Ubuntu本机，不能是SSH远程）
  
  ```bash
  python -m lerobot.teleoperate \
      --robot.ip_address_left="localhost" \
      --robot.port_left=12346 \
      --robot.ip_address_right="localhost" \
      --robot.port_right=12345 \
      --robot.type=enpei_follower \
      --robot.id=enpei_follower \
      --robot.cameras="{ handeye_left: {type: opencv, index_or_path: 4, width: 320, height: 240, fps: 30}, handeye_right: {type: opencv, index_or_path: 0, width: 320, height: 240, fps: 30}, fixed: {type: opencv, index_or_path: 2, width: 320, height: 240, fps: 30}}" \
      --teleop.type=enpei_leader \
      --teleop.port_left=/dev/ttyACM1 \
      --teleop.port_right=/dev/ttyACM0 \
      --teleop.id=enpei_leader \
      --fps=30\
      --display_data=true \
      --enpei_speed_mode=record
  ```
  
  一切正常的话，理应打开下图窗口：
  
  ![](https://enpei-md.oss-cn-hangzhou.aliyuncs.com/202509161119685.png?x-oss-process=style/wp)