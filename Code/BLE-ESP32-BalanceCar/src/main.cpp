#include "Arduino.h"
#include <MPU6050_tockn.h> //磁罗盘库
#include <Wire.h>          //I2C库
#include "SimpleFOC.h"     //电机库
#include "BluetoothSerial.h"
/*--M创动工坊原创，部分零部件本店有配：mcdgf.taobao.com，感谢您的支持--*/
/*----定义两个电机的磁编码器连接引脚和陀螺仪引脚----*/
#define M0_I2C_SDA 21
#define M0_I2C_SCL 22
#define M1_I2C_SDA 19
#define M1_I2C_SCL 18
#define MOVING_AVERAGE_WINDOW_SIZE 5 // 定义移动平均滤波的窗口大小（例如，取前 n 个数据点的平均）

float Balance_Angle_raw = 0.54;                            // 机械平衡角度
const int leftMotorOffset = 0.06, rightMotorOffset = 0.07; // 左右轮的启动扭矩值，pm到一定电压马达才开的转动.
float ENERGY = 3;                                          // 前进后退倾角，控制进后退速度
float kp = 8.8, ki = 0.19, kd = 0.29;                      // 根据调试设置kp ki kd的默认值，kp:8.8   ki:0.19   kd:0.29(平衡小车)
float turn_kp = 0.1;                                       // 转向kp值
float Keep_Angle, bias, integrate;                         // 保持角度，角度偏差，偏差积分变量
float AngleX, GyroX, GyroZ;                                // mpu6050输出的角度值为浮点数，两位有效小数
float vertical_PWM, turn_PWM, PWM, L_PWM, R_PWM;           // 各种PWM计算值
float turn_spd = 0;                                        // 转向Z角速度值，初始值为0
float turn_ENERGY = 300;                                   // 转向Z角速度增加值值
float gyroZBuffer[MOVING_AVERAGE_WINDOW_SIZE];             // 全局数组用于存储最近的 n 个角速度值
int bufferIndex = 0;

MagneticSensorI2C sensor_0 = MagneticSensorI2C(AS5600_I2C); // 初始化AS5600传感器0
TwoWire I2C_0 = TwoWire(0);

MagneticSensorI2C sensor_1 = MagneticSensorI2C(AS5600_I2C); // 初始化AS5600传感器1
TwoWire I2C_1 = TwoWire(1);

MPU6050 mpu6050(I2C_0); // Wire只有两个接口，但I2C可以走总线的方式，所以将MPU6050和AS5600接到同一组SDA、SCL线上，是完全可以的

BLDCMotor motor_0 = BLDCMotor(7);                         // 定义直流无刷电机0，极对数为7
BLDCDriver3PWM driver_0 = BLDCDriver3PWM(25, 32, 33, 12); // 定义电机驱动PWM接的引脚和使能引脚12

BLDCMotor motor_1 = BLDCMotor(7); // 定义直流无刷电机1，极对数为7
BLDCDriver3PWM driver_1 = BLDCDriver3PWM(14, 27, 26, 13);

// 电机转动目标变量定义，这里写的是角度，实际是按扭矩
float target_angle_0 = 0; // 电机角度/速度/扭矩
float target_angle_1 = 0;

char flag = 's'; // 控制左转右转的标签

BluetoothSerial SerialBT; // 实例化蓝牙

void sensor_init()
{
    I2C_0.begin(M0_I2C_SDA, M0_I2C_SCL, 100000); // 定义传感器0接线引脚
    sensor_0.init(&I2C_0);
    I2C_1.begin(M1_I2C_SDA, M1_I2C_SCL, 100000); // 定义传感器1接线引脚
    sensor_1.init(&I2C_1);
    mpu6050.begin();               // 引脚已经定义，即I2C_0的引脚
    mpu6050.calcGyroOffsets(true); // 自动校正打开
}

void serial_debug() // 蓝牙串口调试函数，在线调试PID值
{
    if (SerialBT.available() > 0)
    {
        char DATA = SerialBT.read();
        delay(5);
        switch (DATA)
        {
        /*---机械平衡角度调整-----*/
        case 'u':
            Keep_Angle += 0.01;
            break;
        case 'd':
            Keep_Angle -= 0.01;
            break;
        /*----直立平衡PID调整-----*/
        case '0':
            kp -= 0.1;
            break;
        case '1':
            kp += 0.1;
            break;
        case '2':
            ki -= 0.01;
            break;
        case '3':
            ki += 0.01;
            break;
        case '4':
            kd -= 0.01;
            break;
        case '5':
            kd += 0.01;
            break;
        case '6':
            turn_kp -= 0.01;
            break;
        case '7':
            turn_kp += 0.01;
            break;

        /*-----控制程序-----*/
        case 's':
            flag = 's';
            Keep_Angle = Balance_Angle_raw;
            turn_spd = 0;
            break; // 调节物理平衡点为机械平衡角度值，原地平衡
        case 'f':  // 前进
            flag = 'f';
            Keep_Angle = Balance_Angle_raw + ENERGY;
            turn_spd = 0;
            break;
        case 'b': // 后退
            flag = 'b';
            Keep_Angle = Balance_Angle_raw - ENERGY;
            turn_spd = 0;
            break;
        case 'z': // 不转向
            flag = 'z';
            turn_spd = 0;
            break;
        case 'l': // 左转
            flag = 'l';
            turn_spd = turn_ENERGY;
            break;
        case 'r': // 右转
            flag = 'r';
            turn_spd = -turn_ENERGY;
            break;
        }
        if (kp < 0)
            kp = 0;
        if (ki < 0)
            ki = 0;
        if (kd < 0)
            kd = 0;

        SerialBT.print("Keep_Angle:");
        SerialBT.println(Keep_Angle);
        SerialBT.print("   kp:");
        SerialBT.print(kp);
        SerialBT.print("   ki:");
        SerialBT.print(ki);
        SerialBT.print("   kd:");
        SerialBT.println(kd);
        SerialBT.print("  turn_kp:");
        SerialBT.println(turn_kp);
        SerialBT.println("--------------------");
    }
}

// 移动平均滤波函数
float movingAverageFilter(float newValue)
{
    // 将新值添加到缓冲区
    gyroZBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_WINDOW_SIZE;

    // 计算平均值
    float sum = 0.0;
    for (int i = 0; i < MOVING_AVERAGE_WINDOW_SIZE; ++i)
    {
        sum += gyroZBuffer[i];
    }
    return sum / MOVING_AVERAGE_WINDOW_SIZE;
}

// 转向的PWM
void angle_pwm_calculation()
{ 
    GyroZ = mpu6050.getGyroZ();
    float filteredGyroZ = movingAverageFilter(GyroZ); // 对角速度值进行移动平均滤波
    turn_PWM = turn_kp * (turn_spd - filteredGyroZ);  
}

void verical_pwm_caculation()
{                                                           // 直立PID计算PWM
    AngleX = mpu6050.getAngleX();                           // 陀螺仪获得X方向转动角度
    GyroX = mpu6050.getGyroX();                             // 陀螺仪获得X方向角速度
    bias = AngleX - Keep_Angle;                             // 计算角度偏差，bias为小车角度与结构静态平衡角度的差值
    integrate += bias;                                      // 偏差的积分，integrate为全局变量，一直积累
    integrate = constrain(integrate, -1000, 1000);          // 限定误差积分的最大最小值
    vertical_PWM = kp * bias + ki * integrate + kd * GyroX; // 得到PID调节后的值
    /*=---通过陀螺仪返回数据计算，前倾陀螺仪X轴为正，后仰陀螺仪X轴为负。前倾车前进，后仰车后退，保持直立。但可能为了直立，车会随时移动。*/
}

void motor_init()
{
    /*----电机0初始化-----*/
    motor_0.linkSensor(&sensor_0);
    driver_0.voltage_power_supply = 12;
    driver_0.voltage_limit = 6;
    driver_0.init();
    motor_0.phase_resistance = 2.9;                         // 2.9 Ohms
    motor_0.torque_controller = TorqueControlType::voltage; // 电机的扭矩通过控制电压来实现
    motor_0.linkDriver(&driver_0);
    motor_0.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_0.controller = MotionControlType::torque; // 闭环扭矩控制
    motor_0.PID_velocity.P = 0.108;                 // 速度P值，这个值不能填太大，否则容易抖动
    motor_0.PID_velocity.I = 0.001;                 // 这个值越大，响应速度会慢下来
    motor_0.LPF_velocity.Tf = 0.01f;                // 滤波
    motor_0.P_angle.P = 30;                         // 位置PID的P值
    motor_0.velocity_limit = 20;
    motor_0.useMonitoring(Serial);
    motor_0.init();
    motor_0.initFOC();

    /*----电机1初始化-----*/
    motor_1.linkSensor(&sensor_1);
    driver_1.voltage_power_supply = 12;
    driver_1.voltage_limit = 6;
    driver_1.init();
    motor_1.phase_resistance = 2.9; // 2.9 Ohms，电机相电阻
    motor_1.torque_controller = TorqueControlType::voltage;
    motor_1.linkDriver(&driver_1);
    motor_1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_1.controller = MotionControlType::torque; // 闭环扭矩控制
    motor_1.PID_velocity.P = 0.105;                 // 速度P值，这个值不能填太大，否则容易抖动
    motor_1.PID_velocity.I = 0.001;
    motor_1.LPF_velocity.Tf = 0.01f; // 滤波
    motor_1.P_angle.P = 30;          // 位置PID的P值
    motor_1.velocity_limit = 20;
    motor_1.useMonitoring(Serial);
    motor_1.init();
    motor_1.initFOC();
}

void setup()
{
    Serial.begin(115200);
    SerialBT.begin("ESP32 car"); // 蓝牙设备的名称
    sensor_init();               // 编码器和磁罗盘初始化
    motor_init();                // motor_0,motor_1电机初始化

    /*--------------------------------------------------------*/
    Serial.begin(115200);
    Serial.println("Motor ready!");
    Keep_Angle = Balance_Angle_raw; // 平衡度初始化为结构静态平衡时的陀螺仪角度。Keep Angle可以改变
    delay(1000);
    Serial.println("BLE:ESP32 Car!");
}

void loop()
{
    serial_debug();   // 串口PID调试
    mpu6050.update(); // 陀螺仪刷新
    sensor_0.update();
    sensor_1.update();
    verical_pwm_caculation(); // 直立PWM计算
    PWM = vertical_PWM;       // 电机扭矩值等于经PID计算后的值
    angle_pwm_calculation();  // 转向PWM计算
    motor_0.loopFOC();
    motor_1.loopFOC();
    PWM = constrain(PWM, -60, 60); // 此时PWM还是角度值,限定在-60到60之间
    if (PWM > 0)
    {
        L_PWM = PWM + leftMotorOffset; // 加上电机死区扭矩
        R_PWM = PWM + rightMotorOffset;
    }
    if (PWM < 0)
    {
        L_PWM = PWM - leftMotorOffset; // 反向转动也需要加上电机死区
        R_PWM = PWM - rightMotorOffset;
    }
    // L_PWM -= turn_PWM; // 加上转动角速度值
    // R_PWM += turn_PWM;
    L_PWM = constrain(L_PWM, -200, 200);
    R_PWM = constrain(R_PWM, -200, 200);
    if (AngleX > 45 || AngleX < -45)
    { // 小车倾角过大，已倒下，停止转动；
        motor_0.move(0);
        motor_1.move(0);
    }
    else
    {
        target_angle_0 = static_cast<float>(L_PWM) / 180 * 3.14; // 把陀螺仪MPU6050获得的角度转换为弧度赋值给电机
        target_angle_1 = static_cast<float>(R_PWM) / 180 * 3.14;
        if (PWM > 0)
        {
            target_angle_0 = target_angle_0 + leftMotorOffset; // 加上电机死区扭矩
            target_angle_1 = target_angle_1 + rightMotorOffset;
        }
        if (PWM < 0)
        {
            target_angle_0 = target_angle_0 - leftMotorOffset; // 反向转动也需要加上电机死区
            target_angle_1 = target_angle_1 - rightMotorOffset;
        }
        motor_0.move(target_angle_0); // 实际这是扭矩值
        motor_1.move(target_angle_1);
    }
    // Serial.print(L_PWM);
    // Serial.print("<-L-----R->");
    // Serial.print(R_PWM);
    // Serial.print("-----X倾角->");
    // Serial.print(mpu6050.getAngleX());
    // Serial.print("-----turn_PWM->");
    // Serial.println(turn_PWM);
    // delay(100);
}