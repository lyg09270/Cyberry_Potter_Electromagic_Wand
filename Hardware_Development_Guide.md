# 模块硬件开发指南

#### 模块的连接方式

模块使用金手指与主板连接，以下是金手指的原理图

<img src=".\Photos\Slot.jpg" alt="Spell Card" width="400" height="auto">

图中标注了金手指与STM32的哪些引脚进行了连接以及其复用功能。



<img src=".\Photos\Module_PCB.jpg" alt="Spell Card" width="300" height="auto">

上图是我已经在模块的基础工程中画好的金手指，你只需要添加元件然后连接原理图、布线即可

你可以在./Hardware/Module_DIY.eprj 嘉立创工程文件中对模块进行编辑

<img src=".\Photos\Module_Proj.jpg" alt="Spell Card" width="400" height="auto">

图中左侧对应不同编号的模块（例如：Module4代表4号模块，编写软件时与模块编号要对应）

#### 模块检测原理：

模块检测使用了ADC对模块上分压电阻的电压值进行采样，不同的阻值对应了不同的模块编号（已经在基础工程中添加）



#### 模块引脚介绍：

PA4_CSS: 可作为SPI1的片选引脚或普通IO口

PA5_SCK: 可作为SPI1的时钟引脚或普通IO口

PA6_MISO: 可作为SPI1的发数据引脚或普通IO口

PA7_MOSI: 可作为SPI1的收数据引脚或普通IO口

PB0_CUS: 普通IO口，也可以复用为ADC或TIM3_CH3输出

PB1_ADC: 模块检测使用的ADC引脚，不可修改

PB10_SCL_TX: 可作为IIC2的时钟引脚或USART3的发送引脚或普通IO口

PB11_SDA_RX:  可作为IIC2的数据引脚或USART3的接收引脚或普通IO口



### 如何开发：

1. 确定你的需求

   - 例如你想制作一个Wi-Fi模块在施放法术时与其他设备通讯

2. 芯片选型

   - 选择符合你需求的芯片，并确认与STM32的通讯方式（SPI1、IIC2、USART3）

3. 绘制原理图

   - 按照芯片手册的典型应用和金手指兼容的通讯口绘制原理图

4. PCB 布线

5. 下单PCB 采购元件

   

