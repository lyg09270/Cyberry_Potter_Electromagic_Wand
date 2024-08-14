# 模块软件开发指南

#### 软件整体运行逻辑

编写中.................................



#### 模块开发指南

在你完成了模块的原理图绘制后，你就可以开始设计模块使用的软件了

<img src=".\Photos\Module_Files.jpg" alt="Spell Card" width="300" height="auto">

上图中的文件是你需要在开发中需要编写的文件

<img src=".\Photos\Module3.jpg" alt="Spell Card" width="300" height="auto">

`Module3_Init()`

此函数会在ADC检测到3号模块后进行的初始化操作，你需要在此处初始化你需要使用的GPIO、SPI、IIC、USART

`Module3_Mode0_Handler`

此函数是在模式0（上电初始化后的默认模式）下的操作，这个函数会在CNN模型得到输出后被调用，如果需要使用模型的输出，你需要

 `extern volatile Model_Output_t model_output;`

在这个文件中加入这一行，从而使用main.c中定义的模型输出，模型输出是动作名的枚举，你可以在CyberryPotter.h文件中查看

`Module3_Mode1_Handler`

在切换模式（长按按键0.5s后放开按键）时，`System_Mode` 会在`SYSTEM_MODE_0` 和 `SYSTEM_MODE_1`之间切换，在进入模式1后，此函数也会在CNN模型得到输出后调用



#### 已经存在的初始化函数

我在编写软件时也编写了SPI1,IIC2,USART3的初始化函数，你可以在SPI.h , IIC.h , USART.h 这三个文件中找到这些函数，并在所需要使用的模块文件中添加这些头文件引用

<img src=".\Photos\IIC.jpg" alt="Spell Card" width="700" height="auto">

<img src=".\Photos\SPI.jpg" alt="Spell Card" width="700" height="auto">