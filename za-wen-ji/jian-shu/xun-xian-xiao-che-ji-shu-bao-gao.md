# 寻线小车技术报告

> 总体来讲，整个算法的设计都基于PID动态控制

#### 寻线算法:

假设我们有三个寻线传感器分别叫做 A、B、C。

如果小车处于最佳状态，error == 0，那么A、B、C就应该等于0,1,0。

如果小车向右偏离黑线，error == -1, 那么A、B、C就应该等于1,0,0。

如果小车向左偏离黑线，error == 1, 那么A、B、C就应该等于0,0,1。

#### 控制方法:

**经过无数的尝试，发现只有用差速转弯的方式，才能完美的贴合曲线。**

当 error == 0， 左右轮速度相等，假设都为 50% PWM。

若 error == -1，右轮速度应快于左轮，才能将小车拉回赛道，所以右轮 60% PWM、左轮 40 % PWM。

若 error == 1，左轮速度应快于右轮，才能将小车拉回赛道，所以左轮 60% PWM、右轮 40 % PWM。

> We will keep calculating the errors, and we will move the car according to the errors. So in this way, the car will always stay on the black line.

#### 伪代码看起来像这样:

```c
int A = 0;
int B = 0;
int C = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;
float error=0, P_value = 0, I_value = 0, D_value = 0, PID_value = 0;
float previous_error = 0;

int initial_motor_power = 50;

void calculate_PID() {
    P_value = error;
    I_value = I_value + error;
    D_value = error - previous_error;
    PID_value = (Kp * P_value) + (Ki * I_value) + (Kd * D_value);
    previous_error = error;
}

void setup() {
}

void loop() {
    if (A == 0 && B == 1 && C == 0) {
        error = 0;
    } else if (A == 1 && B == 0 && C == 0) {
        error = -1;
    } else if (A == 1 && B == 0 && C == 0) {
        error = 1;
    }

    calculate_PID();

    if (round(PID_value * 10) == 0) {
        go_straight(initial_motor_power, initial_motor_power);
    }
    else if (PID_value < 0) {
        speed1 = initial_motor_power - abs(PID_value)
        speed2 = initial_motor_power + abs(PID_value)
        constrain(speed1, 0, 100)
        constrain(speed2, 0, 100)
        go_right(speed1, speed2) // speed1 < speed2 here
    }
    else if (PID_value > 0) {
        speed1 = initial_motor_power + abs(PID_value)
        speed2 = initial_motor_power - abs(PID_value)
        constrain(speed1, 0, 100)
        constrain(speed2, 0, 100)
        go_left(speed1, speed2) // speed1 > speed2 here
    }
}
```

## 下面是重复内容

503陆空两栖机器人巡航 项目（冠/亚/季军）技术报告  
南京铁道职业技术学院 胡英杰，孙雅斌  
指导教师: 杨杰

摘要 通过PID算法，不断测试，寻出最优参数，从而达到最好的效果  
关键词 PID；树莓派；差速转弯；智能小车；Pix无人机

1.引言  
现今是一个不断发展的年代，一切过时的东西都将消失在岁月的尘埃之中。唯有那些不断开拓的人，才能立足自我，不断创新，从而在激烈的社会竞争中取得不败之地。  
有时候我想想都能笑出来，接近2020年，有些人还在拿着51单片机做开发。51单片机能联网吗？51单片机支持面向对象编程吗？51单片机可以使用成千上万的开源库吗？这种现象只能说明大部分的人是不思进取的，不愿前进的。  
而本队恰恰相反，从一开始就立足最新科技成果，采用了树莓派+Px4这种高级的组合。且不说树莓派自带的网络属性\(暗指你可以通过 IP packets 的方式对它进行控制\)，就算凭着它深度集成的 Python 开发环境，就能从软件开发上占足优势\(毕竟Python 1分钟, C 写2小时不是瞎说的\)  
另外，由于采用了开源飞控 PX4，我们不需要花大量的精力进行飞机平衡调参，省下来的不少时间可以全用在对赛道本身的研究上。  
如此一来，事情就变得美妙而简单了。

2.作品的总体设计  
2.1 小车部分  
\(1\) 小车的设计思想  
轻: 不轻就飞不起来了  
机动性强: 轮子反应要快，马力要足

\(2\) 小车的组成  
红外巡线模块→树莓派→电机驱动  
超声波模块→树莓派→电机驱动

带孔铝合金自制框架  
树莓派3b+  
TT减速电机  
数字巡线模块  
超声波传感器  
自制万向轮  
轮胎  
电机驱动板  
LED灯

2.2 无人机部分  
\(1\)选购原则  
尽量购买开源产品，因为资源丰富，容易上手。

\(2\)硬件组成  
Pixhawk2.4.8飞控  
F330机架  
好盈乐电调  
YH2212电机  
减震架  
8045浆  
乐迪遥控器  
4000mh电池

2.3 程序设计  
\(1\) 小车巡线算法  
假设我们有三个寻线传感器分别叫做 A、B、C。  
如果小车处于最佳状态，error == 0，那么A、B、C就应该等于0,1,0。  
如果小车向右偏离黑线，error == -1, 那么A、B、C就应该等于1,0,0。  
如果小车向左偏离黑线，error == 1, 那么A、B、C就应该等于0,0,1

\(2\)小车控制方法  
经过无数的尝试，发现只有用差速转弯的方式，才能完美的贴合曲线。  
当 error == 0， 左右轮速度相等，假设都为 50% PWM。  
若 error == -1，右轮速度应快于左轮，才能将小车拉回赛道，所以右轮 60% PWM、左轮 40 % PWM。  
若 error == 1，左轮速度应快于右轮，才能将小车拉回赛道，所以左轮 60% PWM、右轮 40 % PWM。  
We will keep calculating the errors, and we will move the car according to the errors. So in this way, the car will always stay on the black line.

\(3\)核心代码

```cpp
int A = 0;
int B = 0;
int C = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;
float error=0, P_value = 0, I_value = 0, D_value = 0, PID_value = 0;
float previous_error = 0;

int initial_motor_power = 50;

void calculate_PID() {
    P_value = error;
    I_value = I_value + error;
    D_value = error - previous_error;
    PID_value = (Kp * P_value) + (Ki * I_value) + (Kd * D_value);
    previous_error = error;
}

void setup() {
}

void loop() {
    if (A == 0 && B == 1 && C == 0) {
        error = 0;
    } else if (A == 1 && B == 0 && C == 0) {
        error = -1;
    } else if (A == 1 && B == 0 && C == 0) {
        error = 1;
    }

    calculate_PID();

    if (round(PID_value * 10) == 0) {
        go_straight(initial_motor_power, initial_motor_power);
    }
    else if (PID_value < 0) {
        speed1 = initial_motor_power - abs(PID_value)
        speed2 = initial_motor_power + abs(PID_value)
        constrain(speed1, 0, 100)
        constrain(speed2, 0, 100)
        go_right(speed1, speed2) // speed1 < speed2 here
    }
    else if (PID_value > 0) {
        speed1 = initial_motor_power + abs(PID_value)
        speed2 = initial_motor_power - abs(PID_value)
        constrain(speed1, 0, 100)
        constrain(speed2, 0, 100)
        go_left(speed1, speed2) // speed1 > speed2 here
    }
}
```

\(4\)流程控制  
实际上，在这个具体的比赛，小车只有两次机会碰到全白，一是刚进隧道时，二是到达起飞点时。  
只要把握住了这两个节点，那整个程序就可以被分为两部分: 寻黑线 和 过隧道。

