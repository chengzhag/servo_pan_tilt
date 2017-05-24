# servo_pan_tilt
基于mpu9250/6050和舵机的两轴云台，采用ebox和C++编程，Stm32f103单片机

# 新增代码

## mpu9250

### MPU9250
> 基于官方dmp库、i2c的9250驱动

## my_math

### void limit(T &num, T limL, T limH)
> 限制某个数的上下界
### void limitHigh(T &num, T limH)
> 限制某个数的上界
### void limitLow(T &num, T limL)
> 限制某个数的下界

## accurate_pwm

### AccuratePwm
> 基于官方标准库的pwm驱动，继承自ebox的pwm
