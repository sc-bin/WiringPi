项目说明
=================================
本项目是在[WiringPi](https://github.com/WiringPi/WiringPi)的基础上修改而来。

安装本项目到开发板上
========
1. 下载到开发板上
```
git clone https://github.com/sc-bin/WiringPi.git
```
2. 编译并安装到板子上
```
cd WiringPi
./build 
```

玩法一: gpio指令
========
安装完成后，在命令行可以输入以`gpio`开头的指令，实现对引脚的控制


查看型号
------
```
gpio -v
```
在命令行输入上面的命令，即可输出板子型号版本信息


查看所有引脚的状态
------
```
gpio pins
```
命令行输入上面的命令，就会输出一张大表格，其中每一列的含义如下
- `Physical` : 板上排针的编号
- `V` : 代表当前引脚的电平状态，1为高电平，0为低电平
- `Mode` : 这个引脚当前的状态，IN或OUT表示输入输出，ALT数字表示引脚处于复用功能几，OFF则是初始未设置状态
- `Name` : 引脚名称
- `wPi` : wpi编号，在本项目内各种需要选择引脚的地方使用
- `GPIO` : 芯片内部的gpio编号


查看pwm/uart等引脚的位置
------
```
gpio pin [function]
```
其中`[function]`部分可以选择以下几个选项，会输出一张表格显示具备该功能的引脚的位置
- `pwm`
- `uart`
- `spi`
- `i2c`

例如我想知道板子旁边那堆引脚里，有哪些是硬件pwm引脚，则可以输入以下命令，会输出一个表格
```
gpio pin pwm
```

设置引脚功能
------
```
gpio mode [WpiNum] [mode]
```
其中`[WpiNum]`是你要控制引脚的wpi编号，`[mode]`是在下面几种中间选择:
- `in` : 输入模式,浮空
- `up` : 输入模式,开启内部上拉
- `down` : 输入模式,开启内部下拉
- `out` : 输出模式
- `off` : 回归初始未使用状态
- `alt2`...`alt5` : 设置引脚为第n个复用功能

例如我想把wpi编号为7的引脚设置为上拉输入，则在命令行输入如下
```
gpio mode 7 up
```


读取引脚输入电平
------
```
gpio read [WpiNum]
```
- `[WpiNum]`是你要控制引脚的wpi编号

例如我想读取wpi编号为7的引脚的输入状态，就在命令行输入如下
```
gpio read 7
```


控制引脚输出
------
```
gpio write [WpiNum] [status]
```
- `[WpiNum]` 是你要控制引脚的wpi编号，
- `[status]` 为1或0。

例如我想让Wpi引脚7输出高电平
```
gpio write 7 1
```


翻转引脚输出电平
------
```
gpio toggle [WpiNum]
```
- `[WpiNum]` 是你要控制引脚的wpi编号，

让一个处于输出模式的引脚，输出的电平翻转，即7号脚本来输出1，执行完这个命令就会变成0，本来是0，执行完就会变成1。
```
gpio toggle 7
```

硬件pwm-占空比控制
------

```
gpio pwm [wpipin] [range] [freq] 
```
- `[wpipin]`是带硬件pwm的引脚的wpi编号
- `[range]`是占空比，取值为0到1000
- `[freq]`是期望的频率，
1. pwm1的频率可以选择2到20k
2. pwm2的频率可以选择2到20k，但若低于400，则pwm3也必须保持相同频率
3. pwm3的频率可以选择2到20k，但若低于400，则pwm2也必须保持相同频率
4. pwm4的频率可以选择400到20k

比如我想输出一个频率1k,占空比千分之50的波形到硬件pwm1所在的3号引脚上,则在命令行输入如下指令
```
gpio pwm 3 50 1000
```

硬件pwm-时间值控制
------
```
gpio pwmt  [wpipin] [high_time] [period_time] 
```
- `[wpipin]`是带硬件pwm的引脚的wpi编号
- `[high_time]` 单个周期内高电平部分的时长，以us为单位，不能大于`[period_time]`
- `[period_time]` 周期的长度，以us为单位
1. pwm1的`[period_time]`可以选择1到50000us（50ms）
2. pwm2的`[period_time]`可以选择1到50000us（50ms），但若高于2500（2.5ms），则pwm3也必须保持相同周期
3. pwm3的`[period_time]`可以选择1到50000us（50ms），但若高于2500（2.5ms），则pwm2也必须保持相同周期
4. pwm4的`[period_time]`可以选择1到2500（2.5ms）

比如我想输出一个周期20ms,高电平长1.5ms的波形来控制舵机,舵机连接在pwm1所在的3号引脚上,则在命令行输入如下指令
```
gpio pwmt 3 1500 20000
```
