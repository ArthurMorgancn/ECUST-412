#引入包含位姿传感器和电机驱动的库函数
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

#from seekfree import IMU963RA
from seekfree import MOTOR_CONTROLLER
#from seekfree import TSL1401
from seekfree import WIRELESS_UART
#from seekfree import KEY_HANDLER


from display import *
#from imu_get_and_kalman_filter import *
from move_ctrl import *
#from inertial navigation import *

import gc
import time
import math
import array

led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)
beep = Pin('D24' , Pin.OUT, pull=Pin.PULL_UP_47K, value=0)
state2  = switch2.value()
Bat_ADC = ADC('B12')
#编码器初始化
encoder_ur = encoder("C0" , "C1" , False)
encoder_md = encoder("D13" , "D14" , False)#True和False调整编码器方向
encoder_ul = encoder("C2" , "C3" , False)

# 定义片选引脚
cs = Pin('B29' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏 不传入参数就使用当前的 背景色 清屏
# 传入 RGB565 格式参数会直接把传入的颜色设置为背景色 然后清屏
lcd.clear(0x0000)



#UART初始化
uart3 = UART(7)
uart3.init(115200)
print("ok")


#电机初始化

#电机初始化
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = False)
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)#True和False调整电机方向

#pur_speed = 200
speed_ul = 0
speed_ur = 0
speed_md = 0
motor_duty_max = 30

#电池预警
count1 = 0
adc_agg = 0.00
adc_avg = 0.00
    
pur_md_speed,pur_ul_speed,pur_ur_speed = 0,0,0
en_ul_data,en_ur_data,en_md_data = 0,0,0
ticker1_flag = False
ticker1_count = 0
#定时中断1
def time_pit_handler_1(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    ticker1_flag = True  # 否则它会新建一个局部变量


ticker2_flag = False
ticker2_count = 0
#定时中断2
def time_pit_handler_2(time):
    global ticker2_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global pur_speed,integral_md,inlast_md,speed_md,integral_ul,inlast_ul,speed_ul,integral_ur,inlast_ur,speed_ur,kp_md,ki_md,kd_md,kp_ul,ki_ul,kd_ul,kp_ur,ki_ur,kd_ur,v_limit
    global count1,adc_agg,adc_avg
    global pur_md_speed,pur_ul_speed,pur_ur_speed,en_ul_data,en_ur_data,en_md_data
    ticker2_flag = True  # 否则它会新建一个局部变量

    en_ul_data = encoder_ul.get()
    if en_ul_data >= 300:
        en_ul_data = 300
    elif en_ul_data <= -300:
        en_ul_data = -300
    en_ur_data = encoder_ur.get()
    if en_ur_data >= 300:
        en_ur_data = 300
    elif en_ur_data <= -300:
        en_ur_data = -300
    en_md_data = encoder_md.get()
    if en_md_data >= 300:
        en_md_data = 300
    elif en_md_data <= -300:
        en_md_data = -300
    #pur_md_speed,pur_ul_speed,pur_ur_speed = speed_get(100,100,0)
    #pid计算
    integral_md,inlast_md,speed_md = v_pid(pur_md_speed,en_md_data,kp_md,ki_md,kd_md,integral_md,inlast_md,v_limit)
    integral_ul,inlast_ul,speed_ul = v_pid(pur_ul_speed,en_ul_data,kp_ul,ki_ul,kd_ul,integral_ul,inlast_ul,v_limit)
    integral_ur,inlast_ur,speed_ur = v_pid(pur_ur_speed,en_ur_data,kp_ur,ki_ur,kd_ur,integral_ur,inlast_ur,v_limit)
    #电机输出
#     motor_md.duty(speed_md)
    motor_ul.duty(speed_ul)     
    motor_ur.duty(speed_ur)
#     print(en_ur_data,en_ul_data,en_md_data)
#     motor_md.duty(5000)
#     motor_ul.duty(5000)     
#     motor_ur.duty(5000)

      #电池预警
#     adc_value = Bat_ADC.read_u16() * 11 /19520
#     if adc_value<12.00 and adc_value>11.00:
#         count1 = count1 + 1
#         adc_agg = adc_agg + adc_value
#     if count1 == 10:
#         adc_avg = adc_agg / 10
#         if adc_avg<=11.297:
#             beep.high()
#         elif adc_avg>11.297:
#             beep.low()
#         count1 = 0
#         adc_agg = 0.00
#    adc_val = str(adc_avg)
#     #数据传输
    
#     uart3.write(str(120)+","+str(0)+"\r\n")
#     uart3.write(str(pur_md_speed)+","+str(en_md_data)+"\r\n")
    print(en_md_data)
#    uart3.write(adc_val+"\r\n")


pit1 = ticker(1)
pit2 = ticker(2)
# 关联采集接口
#pit1.capture_list() 
pit2.capture_list(encoder_ul, encoder_ur,encoder_md)

# 关联 Python 回调函数
pit1.callback(time_pit_handler_1)
pit2.callback(time_pit_handler_2)

# 每5ms触发一次回调函数
pit1.start(2)
pit2.start(40)


while True:
    
    buf_len = uart3.any()
    if(buf_len):
        buf = uart3.read(buf_len)
        print("uart3 buf_len ={:>6d}".format(buf_len))
        
    
    if (ticker2_flag):
        ticker2_flag = False
        ticker2_count = ticker2_count + 1
        # 定期进行数据解析
    
        
    lcd.str12(0,  30, "kp_ur={:f}".format(kp_ur), 0xF800)
    lcd.str12(0,  40, "ki_ur={:f}".format(ki_ur), 0xF800)
    lcd.str12(0,  50, "kd_ur={:f}".format(kd_ur), 0xF800)
    lcd.str12(0,  60, "kp_ul={:f}".format(kp_ul), 0xF800)
    lcd.str12(0,  70, "ki_ul={:f}".format(ki_ul), 0xF800)
    lcd.str12(0,  80, "kd_ul={:f}".format(kd_md), 0xF800)
    lcd.str12(0,  90, "kp_md={:f}".format(kp_md), 0xF800)
    lcd.str12(0,  100, "ki_md={:f}".format(ki_md), 0xF800)
            
    # send_oscilloscope 将最多八个通道虚拟示波器数据上传到逐飞助手
    # 不需要这么多数据的话就只填自己需要的 只有两个数据就只填两个参数
#     wireless.send_oscilloscope(
#         data_wave[0],data_wave[1],data_wave[2],data_wave[3],
#         data_wave[4],data_wave[5],data_wave[6],data_wave[7])   
        # 翻转 C4 LED 电平
        #led.toggle()
        #print("Ticker trigger {:>6d}.".format(ticker_count))
    
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break
    
    # 回收内存
    gc.collect()

