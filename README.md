# 关于项目

# 项目目前已经完成,量产了一小批，如果需要购买硬件可以在下方找到QQ群加入



### 9.4日更新

- 3D_Print文件夹（添加了Shell2_Laser.stl用于增加红外头，如果你需要）
- Software文件夹
  - 新增CNN目录下的训练数据集更新了模型
  - 调整了部分keil工程内的代码
  - 增加了激光头的代码（在LED.h中解注释以启用这部分代码）
  - 为适配格力空调的协议将红外录制的超时退出改为50ms

### 8.14日更新

- Hardware文件夹 （PCB制版文件、原理图、BOM）
- 3D_Print文件夹（重新调整过的3D打印模型）
- Software文件夹的软件部分还没有彻底完成

#### 文档更新:

- [模块硬件开发指南](./Hardware_Development_Guide.md)

- [模块软件开发指南](./Sotfware_Development_Guide.md)



### 这个项目是什么

此项目是Cyberry Potter Electromagic wand赛博魔杖，你可以使用这个项目中的文件，制作一个赛博魔杖。如果你有足够的能力，你也可以对魔杖的的法术（功能）进行修改，魔杖采用了模块化的设计，不同的模块对应了不同的执行功能。也可以对激活法术的动作进行修改，只需要重新收集数据并训练模型就可以了。



项目使用的嵌入式机器学习库来自[nnom](https://github.com/majianjia/nnom)

此项目包含

- PCB制板文件 (Hardware)
- 3D打印文件 (3D_Print)
- 用于魔杖的软件(Software)
- 动作识别模型训练脚本(Software/CNN)
- 动作的卡片(Software\CNN\SpellsCard)

<img src=".\Software\CNN\SpellsCard/Spell_Card.jpg" alt="Spell Card" width="200" height="auto">

##### 项目视频教程：[bilibili](https://m.bilibili.com/video/BV187pUeKEMr?buvid=XUB1E20B2F44D29D71F181695C9E7F83B9E21&is_story_h5=false&mid=rZPDj1CLrJQMczPBIKGRQw%3D%3D&plat_id=147&share_from=ugc&share_medium=android&share_plat=android&share_session_id=a18d49a9-681d-4397-9cd8-edce4fd64c8c&share_source=GENERIC&share_tag=s_i&timestamp=1724387723&unique_k=6tcr5N7&up_id=34271655)

立创开源平台：[赛博魔杖_STM32卷积神经网络](https://oshwhub.com/lyg0927/cyberwand-stm32-convolutional-ne)

在使用过程中出现问题可以在Bilibili私信我

或者联系我的邮箱：1308770968@qq.com

魔杖技术交流群QQ：698619917


# 魔杖使用前需要知道的事
主板在上电以后会进入模式0，长按按键0.5秒后松开，就可以进入模式1。
任何模式下短按按键并松开：对IMU进行1.5s的采样，将数据输入模型得到动作识别输出。

在红外模块插入时

模式0：在得到动作识别输出后，模块会根据识别到的动作发送已经录制的红外信号

模式1：在得到动作识别输出后，模块会根据识别到的动作等待并录制红外信号

模式0和模式1下执行的操作可以根据插入模块的不同而执行不同操作（需要编写代码）

Type-c口的作用：可以用于串口调试和电池充电，在有Type-c连接时，设备会使用Type-c而非电池供电

电源开关：电源开关负责开启或关闭3.3V的供电，在1没有开启电源开关时，电池充电是可以正常进行的，但是Stm32和陀螺仪等不会上电工作

按钮：按钮有长按松开（大于0.5秒后松开）、短按松开（0.5s内松开）两种控制方法。

按钮前的LED：系统状态指示灯，分为10Hz闪烁，5Hz闪烁，2Hz闪烁，常亮，熄灭五种状态

# 如何自己制作这款魔杖

1. 使用Hardware文件夹下的PCB制版文件制作一块电路板
2. 采购BOM表格中所需要的元件
3. 使用3D打印机打印外壳
4. 将程序烧录进电路板
5. 组装上电



#### 如果你觉得自己制作有困难，我也可以帮你完成部分工作，例如

- PCB打板与焊接并烧录程序
- 3D打印外壳

# 文档简介

- [模块硬件开发指南](./Hardware_Development_Guide.md)

- [模块软件开发指南](./Sotfware_Development_Guide.md)

- [动作识别开发指南](Sotfware_Development_Guide.md)

  

# 开发环境配置

### Python环境：（如果你需要训练自己的模型）

- 首先需要安装一个anaconda（请各位大哥大姐从官网下载，不要在网上随便找安装包，求求你们了）

  - 创建一个python3.9环境
  - `conda create --name py39_env python=3.9`
  - 激活环境
  - `conda activate py39_env`
  - 切换到工程目录的CNN文件夹
  - `cd /path/to/your/directory/Cyberry_Potter_Electromagic_Wand-main/Software/CNN`
  - 安装项目依赖
  - `pip install -r requirements.txt`

  

  ##### Python环境配置可能出现的问题：

  `pip install -r requirements.txt`时出现

  'C:\Users\xxx\AppData\Local\Temp\pip-install-vmn8hi4e\nnom_e898a2d1f9a04e84b72bd63c378042ad' did not run successfully. │ exit code: 128 ╰─> See above for output. note: This error originates from a subprocess, and is likely not a problem with pip.

  解决方法：

  1. 在安装依赖时使用网络代理

  2. 从[nnom](https://github.com/majianjia/nnom)链接中下载nnom的压缩包，解压得到nnom-master（从本地安装nnom）

     - 先使用 `pip install nnom-master`（nnom-master存在的位置）

     - 例如你解压到桌面并且anaconda prompt的当前位置在C:/User/xxxx(xxxx是你的当前账户)

     - 此时你需要使用 `pip install Desktop/nnom-master`

     - 随后再安装其他依赖`pip install -r requirements.txt`

       

  ### Keil环境配置：（如果你需要修改程序）

  - keil版本：keil5(请使用keil官网下载的最新版keil否则可能会遇到一些问题)

  - 编译器版本:Arm Compiler6.22

  - 根据你的设备选用ST-Link或其他设备作为调试器

  - 项目在打开keil是可能会需要安装一些库，请根据提示安装

    

#### 需要使用的库的下载链接：(如果你无法使用keil正常安装以下包，可以从链接中下载安装）请选择下方指定的版本下载并安装

1.[CMSIS6.0.0](https://www.keil.arm.com/packs/cmsis-arm/versions/)

2.[CMSIS compiler 2.1.0](https://www.keil.arm.com/packs/cmsis-compiler-arm/versions/)

3.[Stm32F1xx_DFP2.4.1](https://www.keil.arm.com/packs/stm32f1xx_dfp-keil/versions/)

#### 可能存在的问题

编译报错：C:/Users/xxx/AppData/Local/arm/packs/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/misc.c:131:11: error: no member named 'IP' in 'NVIC_Type'

这是项目使用到的STM32F1XX_DFP2.4.1版本库存在的一个问题，这个有问题的文件安装在以下目录，默认是只读的，请在这个目录下将其只读选项取消勾选
C:/Users/xxx/AppData/Local/arm/packs/Keil/STM32F1xx_DFP/2.4.1/Device/StdPeriph_Driver/src/misc.c
    
（AppData文件夹默认是隐藏的，请先设置显示隐藏文件）
    
请将device库中将misc.c文件里的上述代码（131行）修改为
NVIC->IPR[NVIC_InitStruct->NVIC_IRQChannel] = tmppriority
    

# 如何自己制作模块

##### 金手指插槽可以兼容IIC、串口、SPI三种通讯协议，你可以根据自己的需求以及Hardware文件夹中的尺寸标准制作你需要的模块（例如蓝牙和Wi-Fi）

我也准备了一个嘉立创EDA的工程文件（hardware/Module_DIY.eprj），文件中已经准备画好了模块的外形和引脚名称，你可以根据自己的需求添加元件，制作一个符合你想要的模块



# 制作模块后如何进行二次开发

可以在software/module文件夹中对模块modulex.c(你需要的模块编号 例如module4.c)文件进行修改，写入不同模式(Mode0,Mode1)下在识别到动作后所需要对模块执行的命令（例如使用串口与蓝牙芯片通讯，发出信息）

# 如何训练

1. 使用[脚本](https://github.com/lyg09270/CyberryPotter_ElectromagicWand_Basic_Project/blob/main/CNN/Serial_Read.py)收集数据
   - 首先需要将config.h文件中的SYSTEM_MODE_DATA_COLLECT解除注释
   - 完成以上操作后单片机将进入数据打印模式而非推理模式
2. 运行[模型训练](https://github.com/lyg09270/CyberryPotter_ElectromagicWand_Basic_Project/blob/main/CNN/CNNTrainRaw.py)脚本
   - 运行训练脚本你将得到一个.h5的模型文件和一个.h的c头文件
   - .h5文件用于在[串口模型测试脚本](https://github.com/lyg09270/CyberryPotter_ElectromagicWand_Basic_Project/blob/main/CNN/CNNTestSerialRaw.py)对未量化的模型进行测试
   - .h文件是单片机用于编译模型所需要的文件
