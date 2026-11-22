#决赛代码
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

from seekfree import IMU963RX
from seekfree import MOTOR_CONTROLLER
from seekfree import TSL1401
from seekfree import WIRELESS_UART
from seekfree import KEY_HANDLER
from seekfree import DL1X

from display import *
from imu_get_and_kalman_filter import *
from move_ctrl import *
from inertial_navigation_acc import *

import gc
import time
import math
import array

led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state2  = switch2.value()

tof = DL1X()



#编码器初始化
encoder_ur = encoder("C0" , "C1" , False)
encoder_md = encoder("D13" , "D14" , False)#True和False调整编码器方向
encoder_ul = encoder("C2" , "C3" , False)

#UART初始化
uart6 = UART(7)
uart6.init(115200)
wireless = UART(2)
wireless.init(460800)
buf_len = 0
buf = 0
buf_data_x = 899
buf_data_y = 899

buf_wireless_len = 0
buf_wireless = 0
send_wireless_flag = 0


stop_self_flag = 0#本车停止标志位
stop_other_flag = 0#从车停止标志位

#电机初始化
motor_ur = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = False)
motor_md = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5, 13000, duty = 0, invert = False)
motor_ul = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)#True和False调整电机方向

#位姿传感器初始化
imu = IMU963RX()

#蜂鸣器初始化
beep = Pin('D24', Pin.OUT, pull = Pin.PULL_UP_47K, value = False)

#ADC初始化
Bat_ADC = ADC('B12')

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
#电池预警
count1 = 0
adc_agg = 0.00
adc_avg = 0.00

obj_sum = 0


def batter_waring():
  global count1,adc_agg,adc_avg
  adc_value = Bat_ADC.read_u16() * 11 /19520
  if adc_value<12.00 and adc_value>11.00:
        count1 = count1 + 1
        adc_agg = adc_agg + adc_value
  if count1 == 10:
        adc_avg = adc_agg / 10
#         if adc_avg<=11.297:
#             beep.high()
        #elif adc_avg>11.297:
            #beep.low()
        count1 = 0
        adc_agg = 0.00





ticker1_flag = False
ticker1_count = 0

timer_2ms = 0
LPF_ALPHA    = 0.4        # 低通滤波
PI           = 3.141592653589793
# 定义全局滤波变量（放在函数外初始化）
acc_x_filt = 0.0
acc_y_filt = 0.0
acc_z_filt = 0.0
roll_filt = 0.0
pitch_filt = 0.0
# 初始角度
init_angle = 90

dt = 0.004

gyro_x_offset = 0.696234826386301
gyro_y_offset = -6.352734055123758
gyro_z_offset = -4.341108260342145
# PI控制器参数
param_Kp = 0.19
param_Ki = 0.001

# 滤波后数据
acc_data  = [0.0, 0.0, 0.0]
gyro_data = [0.0, 0.0, 0.0]

# 积分项
I_ex = 0.0
I_ey = 0.0
I_ez = 0.0

# 四元数
class Quater:
    def __init__(self):
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
Q_info = Quater()

# 欧拉角
class Euler:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_roll = 0.0
        self.last_yaw = 0.0
        self.Dirchange = 0
eulerAngle = Euler()

#求平方根倒数
def fast_sqrt(x):
    if x <= 0:
        return 0.0
    return 1.0 / math.sqrt(x)

#六轴数据一阶低通滤波
def imu_update_acc(ax, ay, az, gx, gy, gz):
    global acc_data, gyro_data
    acc_data[0]  = LPF_ALPHA * acc_data[0]  + (1-LPF_ALPHA) * ax
    acc_data[1]  = LPF_ALPHA * acc_data[1]  + (1-LPF_ALPHA) * ay
    acc_data[2]  = LPF_ALPHA * acc_data[2]  + (1-LPF_ALPHA) * az
    gyro_data[0] = LPF_ALPHA * gyro_data[0] + (1-LPF_ALPHA) * gx
    gyro_data[1] = LPF_ALPHA * gyro_data[1] + (1-LPF_ALPHA) * gy
    gyro_data[2] = LPF_ALPHA * gyro_data[2] + (1-LPF_ALPHA) * gz
#     gyro_min = 0
#     if abs(gyro_data[0] - gyro_x_offset) <= gyro_min:
#         gyro_data[0] = gyro_x_offset
#     if abs(gyro_data[1] - gyro_y_offset) <= gyro_min:
#         gyro_data[1] = gyro_y_offset
#     if abs(gyro_data[2] - gyro_z_offset) <= gyro_min:
#         gyro_data[2] = gyro_z_offset

#AHRS更新
def ICM_AHRSupdate(gx, gy, gz, ax, ay, az):
    global Q_info, I_ex, I_ey, I_ez

    halfT = 0.5 * dt
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3

    # 归一化加速度
    norm = fast_sqrt(ax*ax + ay*ay + az*az)
    ax *= norm
    ay *= norm
    az *= norm

    # 重力方向估计
    vx = 2 * (q1*q3 - q0*q2)
    vy = 2 * (q0*q1 + q2*q3)
    vz = (q0*q0 - q1*q1 - q2*q2 + q3*q3)

    # 误差计算
    ex = ay * vz - az * vy
    ey = az * vx - ax * vz
    ez = ax * vy - ay * vx

    # 积分限幅
    I_ex += ex * dt
    I_ey += ey * dt
    I_ez += ez * dt
    I_ex = max(-5.0, min(5.0, I_ex))
    I_ey = max(-5.0, min(5.0, I_ey))
    I_ez = max(-5.0, min(5.0, I_ez))

    # PI校正陀螺仪
    gx += param_Kp * ex + param_Ki * I_ex
    gy += param_Kp * ey + param_Ki * I_ey
    gz += param_Kp * ez + param_Ki * I_ez

    # 四元数微分更新
    dq0 = (-q1*gx - q2*gy - q3*gz) * halfT
    dq1 = ( q0*gx + q2*gz - q3*gy) * halfT
    dq2 = ( q0*gy - q1*gz + q3*gx) * halfT
    dq3 = ( q0*gz + q1*gy - q2*gx) * halfT

    # 二阶修正
    delta_2 = (2*halfT*gx)**2 + (2*halfT*gy)**2 + (2*halfT*gz)**2
    q0 = (1 - delta_2/8) * q0 + dq0
    q1 = (1 - delta_2/8) * q1 + dq1
    q2 = (1 - delta_2/8) * q2 + dq2
    q3 = (1 - delta_2/8) * q3 + dq3

    # 四元数归一化
    norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    Q_info.q0 = q0 * norm
    Q_info.q1 = q1 * norm
    Q_info.q2 = q2 * norm
    Q_info.q3 = q3 * norm

flag1 = 1
gyro_sum = 0.0
ax_f = 0
ay_f = 0
az_f = 0
gx_f = 0
gy_f = 0
gz_f = 0
alt_sum = 0
alt_count = 0
def altitude_calc():
    global eulerAngle
    global init_angle
    global ax_f, ay_f, az_f, gx_f, gy_f, gz_f
    global alt_count,alt_sum
    # 1. 读取 LSM6DSR 数据
    imu_data = imu.get()
    ax = imu_data[0]
    ay = imu_data[1]
    az = imu_data[2]
    gx = imu_data[3]
    gy = imu_data[4]
    gz = imu_data[5]
    
        
    
    # 2. 低通滤波
    imu_update_acc(ax, ay, az, gx, gy, gz)
    
#     alt_count+=1
#     alt_sum += gyro_data[2]
#     if alt_count == 1000:
#         print(alt_sum/1000)
      
    # 3. 减去陀螺仪零偏 + 单位转换
    gx_f = -(gyro_data[0] - gyro_x_offset) /14.3 * PI / 180.0
    gy_f = -(gyro_data[1] - gyro_y_offset) /14.3 * PI / 180.0
    gz_f = -(gyro_data[2] - gyro_z_offset) /14.3 * PI / 180.0
    ax_f = -acc_data[0]
    ay_f = -acc_data[1]
    az_f = -acc_data[2]


    # 4. AHRS 解算
    ICM_AHRSupdate(gx_f, gy_f, gz_f, ax_f, ay_f, az_f)
    q0 = Q_info.q0
    q1 = Q_info.q1
    q2 = Q_info.q2
    q3 = Q_info.q3

    # 5. 四元数转欧拉角
    eulerAngle.pitch = math.asin(-2 * q1 * q3 + 2 * q0 * q2) * 180.0 / PI
    eulerAngle.roll  = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2*q1*q1 -2*q2*q2 +1) * 180.0 / PI
    eulerAngle.yaw   = math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2*q2*q2 -2*q3*q3 +1) * 180.0 / PI
    eulerAngle.yaw = eulerAngle.yaw + init_angle
    # Yaw 范围限定 0到360
    if eulerAngle.yaw > 360:
        eulerAngle.yaw -= 360
    if eulerAngle.yaw < 0:
        eulerAngle.yaw += 360

    # 大角度保护（防止翻车跳变）
    if abs(eulerAngle.pitch) > 70.0:
        eulerAngle.roll = eulerAngle.last_roll
        eulerAngle.yaw  = eulerAngle.last_yaw

    # 转圈计数
    if (eulerAngle.yaw - eulerAngle.last_yaw) < -350:
        eulerAngle.Dirchange += 1
    elif (eulerAngle.yaw - eulerAngle.last_yaw) > 350:
        eulerAngle.Dirchange -= 1

    eulerAngle.last_roll = eulerAngle.roll
    eulerAngle.last_yaw  = eulerAngle.yaw
    #print(str(eulerAngle.yaw)+"\r\n")



en_md_data = 0
en_ul_data = 0
en_ur_data = 0
def time_pit_handler_1(time):
    global timer_2ms
    global buf_wireless_len,buf_wireless,first_ready_flag,second_ready_flag
    global state, state_index
    global en_md_data,en_ur_data,en_ul_data,X,Y
    global target_pulses_y,target_pulses_x
    if state == state_index["push"]:
        pul_y = 35
        pul_x = 27
    elif state == state_index["push_2nd"]:
        pul_y = 27
        pul_x = 36
    else:
        pul_y = 27
        pul_x = 27
    timer_2ms += 1
       #get_encoders()
    en_limit = 80
    en_ul_data = encoder_ul.get()
    en_ur_data = encoder_ur.get()
    en_md_data = encoder_md.get()
    
    if en_ul_data >= en_limit:
        en_ul_data = en_limit
    elif en_ul_data <= -en_limit:
        en_ul_data = -en_limit
    if en_ur_data >= en_limit:
        en_ur_data = en_limit
    elif en_ur_data <= -en_limit:
        en_ur_data = -en_limit 
    if en_md_data >= en_limit:
        en_md_data = en_limit
    elif en_md_data <= -en_limit:
        en_md_data = -en_limit

    #编码器脉冲累加
    encoder_x = en_md_data * math.cos(math.radians(eulerAngle.yaw-90)) + en_ul_data * math.cos(math.radians(eulerAngle.yaw+150)) + en_ur_data * math.cos(math.radians(eulerAngle.yaw+30))
    encoder_y = en_md_data * math.sin(math.radians(eulerAngle.yaw-90)) + en_ul_data * math.sin(math.radians(eulerAngle.yaw+150)) + en_ur_data * math.sin(math.radians(eulerAngle.yaw+30))
    x_cross, y_cross = update_wheel_pulses(encoder_x,encoder_y,pul_x,pul_y)
    X += x_cross
    Y += y_cross
   

      
  
  
  

ticker2_flag = False
ticker2_count = 0#4ms
t_flag = 0

x_pulses = 0
y_pulses = 0

omega = 90

speed_up_count = 0
detect_count = 0
push_count = 0
find_flag = 0
detect_flag = 0
tof_flag = 0
color_flag = 0
enc_cnt = 0
enc_abs_x = 0
enc_abs_y = 0
tof_data = 8192
end_count = 0
spin_2_cnt = 0
detect_x_filter = 160
stop_count = 0
color_count = 0
home_detect_count = 0

#定时中断2
def time_pit_handler_2(time):
    global ticker2_flag, ticker2_count, speed_up_count,push_count,detect_count,color_count,omega,X,Y
    global move_angle,move_angle_index,turn_speed,en_md_data,en_ul_data,en_ur_data
    global point_get_flag,find_flag,detect_flag,color_flag,buf_data_x,buf_data_y,yellow_detect_flag
    global eulerAngle,init_angle
    global detect_x_filter
    global state,state_index,obj_index,obj
    global ax_f, ay_f, az_f, gx_f, gy_f, gz_f
    global enc_abs_x,enc_abs_y
    global x_x,p_x,x_y,p_y,F_x,F_y,H_x,H_y,Q_x,Q_y,R_x,R_y
    global tof_flag,tof_data,end_count,spin_2_cnt,stop_count,home_detect_count
    global buf_wireless_len,buf_wireless
    global first_ready_flag,second_ready_flag,obj_sum
    global pid_ul,pid_ur,pid_md,pid_theita,pid_point_get,pid_detect,pid_tof,pid_theita_w,target_pulses_x,target_pulses_y

    
    ticker2_count += 1
    
    altitude_calc()
    

    #tof_data = tof.get()

    
    
    #状态判断
    if state == state_index["speed_up"]:#缓启动
        # 计算速度曲线
        speed_up_count += 0.004
        move_angle,target,point_get_flag = goto_point(X,Y,110,60)
        move_speed = sigmoid_speed_curve(speed_up_count, 0, target, 0.4, 6)
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90, gx_f)#计算角度环
    
    elif state == state_index["move"]:
        move_angle,move_speed,point_get_flag = goto_point(X,Y,110,60)
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90, gx_f)
        if point_get_flag:
            move_speed = 0
            turn_speed = 0 
    
    elif state == state_index["detect"]:
        detect_count += 1
        if X > 300:
            state = state_index["home"]
            wireless.write("z")
        if buf_data_x == 899:
            if find_flag == 0:
                if detect_count <= 0.5 * 250:                
                    move_speed = sigmoid_speed_curve(detect_count/250, 0, 35, 0.5, 6)
                else:
                    move_speed = 35
            else:
                move_speed = 10
                
            move_angle = 0
        else:
            find_flag = 1
            detect_x_filter = 0.7 * detect_x_filter + 0.3 * buf_data_x
            
            move_speed_x = pid_detect.compute(0,160 - detect_x_filter)
            tof_data = tof.get()
            distance_scale = min(tof_data / 200, 1.0)
            move_speed_x *= distance_scale
            if tof_data != 8192:
                move_speed_y = pid_tof.compute(0, 90 - tof_data)
            else:
                move_speed_y = 0
            move_speed = math.sqrt(move_speed_x*move_speed_x + move_speed_y*move_speed_y)          # 等价于 sqrt(x^2 + y^2)，更安全
            angle_rad = math.atan2(move_speed_y, move_speed_x)          # 返回 -π 到 π 之间的弧度
            move_angle = math.degrees(angle_rad)   # 转换为度数
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90,gx_f)
        if abs(buf_data_x - 160) <= 25 and tof_data < 120:
            detect_flag += 1
        else:
            detect_flag = 0
            
     
    
    elif state == state_index["spin"]:
#         if second_ready_flag == 0:
#             wireless.write(f"s{int(X)+100}{int(Y)+100}v")
        
        pur_angle = -90
        turn_sum = 2.95
        
        omega = omega - 0.25  
        if eulerAngle.yaw > 180:
            yaw_push = eulerAngle.yaw - 360
        else:
            yaw_push = eulerAngle.yaw
        if yaw_push >= pur_angle:
            turn_speed = pid_theita.compute_theita(0, yaw_push - omega,gx_f)
            move_speed = turn_speed * turn_sum
            move_angle = 0
        else:
            state = state_index["detect_2nd"]
            second_ready_flag = 0
            #uart6.write('1')
            detect_x_filter = 160
            detect_flag = 0
            move_speed = 0
            turn_speed = 0
            pid_ur.inlast = 0
            pid_ur.integral = 0
            pid_ul.inlast = 0
            pid_ul.integral = 0
            pid_md.inlast = 0
            pid_md.integral = 0
            
            
    elif state == state_index["detect_2nd"]:
        detect_count += 1
        if buf_data_x == 899:
            move_speed = 0
            move_angle = 0
        else:
            detect_x_filter = 0.7 * detect_x_filter + 0.3 * buf_data_x
            move_speed_x = pid_detect.compute(0,160 - detect_x_filter)
            tof_data = tof.get()
            distance_scale = min(tof_data / 200, 1.0)
            move_speed_x *= distance_scale
            if tof_data != 8192:
                move_speed_y = pid_tof.compute(0, 40 - tof_data)
            else:
                move_speed_y = 0
            move_speed = math.sqrt(move_speed_x*move_speed_x + move_speed_y*move_speed_y)          # 等价于 sqrt(x^2 + y^2)，更安全
            angle_rad = math.atan2(move_speed_y, move_speed_x)          # 返回 -π 到 π 之间的弧度
            move_angle = math.degrees(angle_rad)   # 转换为度数
        
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)
        
        if abs(buf_data_x - 160) <= 25 and tof_data < 100:
            detect_flag += 1
        else:
            detect_flag = 0
    
        
        
    
    
    elif state == state_index["push"]:
        push_count += 1
        
        if obj == obj_index["ball"]:
            if push_count < 240:
                move_speed = 60
                move_angle = 90
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180,gx_f)            
           
            else:
                state = state_index["spin_2nd"]
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                if obj == obj_index["ball"]:
                    omega = 90
                else:
                    omega = 270
                move_speed = 0
                move_angle = 0
                turn_speed = 0
                push_count = 0
                pid_ur.inlast = 0
                pid_ur.integral = 0
                pid_ul.inlast = 0
                pid_ul.integral = 0
                pid_md.inlast = 0
                pid_md.integral = 0
                
        else:#obj = "bear" or obj = "cube"
            if Y > 80:
                if push_count <= 100:
                    move_speed = sigmoid_speed_curve(push_count / 250, 0, 60, 0.4, 6)
                else:
                    move_speed = 60
          
                move_angle = 90
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)            
            else:
                
                #obj_sum = obj_sum + 1
                state = state_index["spin_2nd"]
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                move_speed = 0
                move_angle = 0
                turn_speed = 0
                push_count = 0
                pid_ur.inlast = 0
                pid_ur.integral = 0
                pid_ul.inlast = 0
                pid_ul.integral = 0
                pid_md.inlast = 0
                pid_md.integral = 0
                
                
    elif state == state_index["spin_2nd"]:

        if obj == obj_index["ball"]:
            omega = (omega - 0.25) % 360
            if eulerAngle.yaw >= 90:
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - omega,gx_f)            
                move_speed = turn_speed * 3.8
                move_angle = 0
            else:
                state = state_index["detect_3rd"]
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                spin_2nd_cnt = 0
                move_speed = 0
                turn_speed = 0
                pid_ur.inlast = 0
                pid_ur.integral = 0
                pid_ul.inlast = 0
                pid_ul.integral = 0
                pid_md.inlast = 0
                pid_md.integral = 0
        
        elif obj == obj_index["cube"]:
            omega = omega - 0.25
            if eulerAngle.yaw >= 180:
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - omega,gx_f)            
                move_speed = turn_speed * 3.0
                move_angle = 0
            else:
                state = state_index["detect_3rd"]
                spin_2nd_cnt = 0
                move_speed = 0
                turn_speed = 0
        else:
            omega = omega + 0.25
            if eulerAngle.yaw > 180:
                yaw_push = eulerAngle.yaw - 360
            else:
                yaw_push = eulerAngle.yaw
            if yaw_push <= 0:
                turn_speed = pid_theita.compute_theita(0, yaw_push - omega + 360,gx_f)            
                move_speed = turn_speed * 3.0
                move_angle = 0
            else:
                state = state_index["detect_3rd"]
                spin_2nd_cnt = 0
                move_speed = 0
                turn_speed = 0
                
     
    elif state == state_index["detect_3rd"]:
        detect_count += 1
        if buf_data_x == 899:
            move_speed = 0
            move_angle = 0
        else:
            detect_x_filter = 0.7 * detect_x_filter + 0.3 * buf_data_x
            move_speed_x = pid_detect.compute(0,160 - detect_x_filter)
            tof_data = tof.get()
            distance_scale = min(tof_data / 200, 1.0)
            move_speed_x *= distance_scale
            if tof_data != 8192:
                move_speed_y = pid_tof.compute(0, 40 - tof_data)
            else:
                move_speed_y = 0
            move_speed = math.sqrt(move_speed_x*move_speed_x + move_speed_y*move_speed_y)          # 等价于 sqrt(x^2 + y^2)，更安全
            angle_rad = math.atan2(move_speed_y, move_speed_x)          # 返回 -π 到 π 之间的弧度
            move_angle = math.degrees(angle_rad)   # 转换为度数
        if obj == obj_index["ball"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90,gx_f)
        elif obj == obj_index["cube"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180,gx_f)
        elif obj == obj_index["bear"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 0,gx_f)
        if abs(buf_data_x - 160) <= 25 and tof_data < 100:
            detect_flag += 1
        else:
            detect_flag = 0
                
                
    elif state == state_index["stop"]:

        move_speed = 0
        if obj == obj_index["cube"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180,gx_f)
        elif obj == obj_index["ball"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)
        else:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 0,gx_f)        
        if first_ready_flag:
            state = state_index["push_2nd"]
            push_count = 0
            buf_wireless_len = 0
            send_wireless_flag = 0
            
    elif state == state_index["push_2nd"]:
        
        push_count += 1
        if obj == obj_index["ball"]:
            if push_count < 50:
                move_speed = 0
                move_angle = 0
                turn_speed = 0
            elif Y < 240 and yellow_detect_flag == 0:
                
                if buf_data_x != 899:
                    move_speed_x = 1.5 * pid_detect.compute(0,160 - buf_data_x)
                else:
                    move_speed_x = 0
        
                move_speed_y = -35
                move_speed = math.sqrt(move_speed_x*move_speed_x + move_speed_y*move_speed_y)          # 等价于 sqrt(x^2 + y^2)，更安全
                angle_rad = math.atan2(move_speed_y, move_speed_x)          # 返回 -π 到 π 之间的弧度
                move_angle = math.degrees(angle_rad)   # 转换为度数
                
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)
            else:
                state = state_index["yellow_detect"]
                move_speed = 0
                move_angle = 0
                turn_speed = 0
                push_count = 0
                
        elif obj == obj_index["bear"]:
            if X < 290 and yellow_detect_flag == 0:
                if push_count <= 100:
                    move_speed = sigmoid_speed_curve(push_count / 250, 0, 40, 0.4, 6)
                else:
                    move_speed = 40
                move_angle = 90
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 0,gx_f)
            else:
                state = state_index["yellow_detect"]
                move_speed = 0
                move_angle = 0
                turn_speed = 0
                push_count = 0
                pid_ur.inlast = 0
                pid_ur.integral = 0
                pid_ul.inlast = 0
                pid_ul.integral = 0
                pid_md.inlast = 0
                pid_md.integral = 0
                
        else:#obj=cube
            if X > 30 and yellow_detect_flag == 0:
                if push_count <= 100:
                    move_speed = sigmoid_speed_curve(push_count / 250, 0, 40, 0.4, 6)
                
                else:
                    move_speed = 40
                    
                move_angle = 90
                turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 190,gx_f)
            else:

                state = state_index["yellow_detect"]
                move_speed = 0
                move_angle = 0
                turn_speed = 0
                push_count = 0
                pid_ur.inlast = 0
                pid_ur.integral = 0
                pid_ul.inlast = 0
                pid_ul.integral = 0
                pid_md.inlast = 0
                pid_md.integral = 0
               
                
                
    elif state == state_index["yellow_detect"]:
        move_speed = 25
        if obj == obj_index["ball"]:
            move_speed = -25
        move_angle = 90
        if obj == obj_index["cube"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180,gx_f)
            y_flag = X < -30
            y2_flag = X < 0
        elif obj == obj_index["bear"]:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 0,gx_f)
            y_flag = X > 350
            y2_flag = X > 320
        else:
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)
            y_flag = Y > 300
            y2_flag = Y > 270
        if (yellow_detect_flag == 1 and y2_flag) or y_flag:
            state = state_index["end"]
#============================================================================================================
            
    
   
    
    
   
    
#     elif state == state_index["stop_2nd"]:
#     
#         move_speed = 0
#         if obj == obj_index["ball"]:
#             turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90,gx_f)
#         else:
#             turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 0,gx_f)
#         
#         if second_ready_flag:
#             state = state_index["push_2nd"]
#             push_count = 0
#             buf_wireless_len = 0
#             buf_wireless = 0
#             send_wireless_flag = 0
        

                
#===========================================================================================================             
    elif state == state_index["end"]:
        end_count += 1
        wait_time = 150
        if obj == obj_index["ball"]:
            wait_time = 500
        if end_count <= wait_time:
            move_speed = 0
            move_angle = 90
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90,gx_f)
        
        else:
            obj_sum += 1
            if obj_sum == 3:
                state = state_index["home"]
            else:
                state = state_index["speed_up"]

            if not yellow_detect_flag:
                if obj == obj_index["cube"]:
                    X = X + 30
                elif obj == obj_index["ball"]:
                    Y = Y + 15
                elif obj == obj_index["bear"]:
                    X = X - 30
            yellow_detect_flag = 0
            omega = 90
            obj_detect_flag = 0
            first_ready_flag = 0
            obj = obj_index["null"]
            end_count = 0
            move_speed = 0
            move_angle = 0
            turn_speed = 0
            
    elif state == state_index["home"]:
        move_angle,move_speed,point_get_flag = goto_point(X,Y,50,30)
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90, gx_f)
        if point_get_flag:
            state = state_index["home_detect"]
            point_get_flag = 0
            yellow_detect_flag = 0
            
    elif state == state_index["home_detect"]:
        home_detect_count += 1
        if home_detect_count < 200: 
            move_speed = 0
            move_angle = 90
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180, gx_f)
        else:
            move_speed = 25
            move_angle = 90
            turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 180, gx_f)
            if yellow_detect_flag:
                state = state_index["back"]
                move_speed = 0
            
    elif state == state_index["back"]:
        move_angle,move_speed,point_get_flag = goto_point(X,Y,50,-90)
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90, gx_f)
        if point_get_flag:
            state = state_index["blind_box"]
        
        
    elif state == state_index["blind_box"]:
        move_speed = 0
        move_angle = 90
        turn_speed = -20
        if eulerAngle.yaw > 100 and eulerAngle.yaw < 110:
            state = state_index["stand_still"]
        
    elif state == state_index["stand_still"]:
        move_speed = 0
        move_angle = 90
        turn_speed = 0
            
#     elif state == state_index["back"]:
#         if buf_data_x == 899:
#             move_speed = 0
#             move_angle = 0
#         else:
#             move_speed_x = pid_detect.compute(0,160 - buf_data_x)
#             tof_data = tof.get()
#             distance_scale = min(tof_data / 200, 1.0)
#             move_speed_x *= distance_scale
#             if tof_data != 8192:
#                 move_speed_y = pid_tof.compute(0, 40 - tof_data)
#             else:
#                 move_speed_y = 0
#             move_speed = math.sqrt(move_speed_x*move_speed_x + move_speed_y*move_speed_y)          # 等价于 sqrt(x^2 + y^2)，更安全
#             angle_rad = math.atan2(move_speed_y, move_speed_x)          # 返回 -π 到 π 之间的弧度
#             move_angle = math.degrees(angle_rad)   # 转换为度数
#         turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 270,gx_f)    
        
    elif state == state_index["test"]:
        #move_angle,move_speed,point_get_flag = goto_point(X,Y,46,300)
        if Y < 210:
            move_speed = 60
        else:
            move_speed = 0
        move_angle = 90
        turn_speed = pid_theita.compute_theita(0, eulerAngle.yaw - 90,gx_f)
        
        if point_get_flag:
            move_speed = 0
            turn_speed = 0 

    #计算三个轮子速度     
    motor_md_speed, motor_ul_speed, motor_ur_speed = speed_get(move_speed,move_angle,turn_speed)

    

    #pid计算
    speed_md = pid_md.compute(motor_md_speed,en_md_data)
    speed_ul = pid_ul.compute(motor_ul_speed,en_ul_data)
    speed_ur = pid_ur.compute(motor_ur_speed,en_ur_data)
    #输出pwm
    motor_ul.duty(speed_ul)
    motor_ur.duty(speed_ur) 
    motor_md.duty(speed_md)






pit1 = ticker(1)
pit2 = ticker(2)
# 关联采集接口
pit1.capture_list(encoder_ul, encoder_ur, encoder_md) 
pit2.capture_list(imu,tof)

# 关联 Python 回调函数
pit1.callback(time_pit_handler_1)
pit2.callback(time_pit_handler_2)

# 每5ms触发一次回调函数
pit1.start(40)
pit2.start(4)


state_index = {
    "test":-2,
    "speed_up":0,
    "move":1,
    "detect":2,
    "tof":3,
    "spin":4,
    "detect_2nd":5,
    "tof_2nd":6,
    "stop":7,
    "push":8,
    "spin_2nd":9,
    "detect_3rd":10,
    "tof_3rd":11,
    "stop_2nd":12,
    "push_2nd":13,
    "end":14,
    "wait":15,
    "home":16,
    "back":17,
    "yellow_detect":18,
    "blind_box":19,
    "stand_still":20,
    "home_detect":21
}
state = state_index["speed_up"]
#state = state_index['test']

obj_index = {
    "cube":0,
    "ball":2,
    "bear":1,
    "null":3
}
obj = obj_index["null"]
obj_detect = [0,0,0]
obj_detect_flag = 0
first_ready_flag = 0
second_ready_flag = 0
yellow_detect_flag = 0
X_save = 0


while True:
    
    #电池预警
    batter_waring()
    
    
    lcd.str12(0,  0, "x={:f}".format(X), 0xF800)
    lcd.str12(0,  10, "y={:f}".format(Y), 0xF800)
    lcd.str12(0,  20, "yaw={:f}".format(eulerAngle.yaw), 0xF800)
    lcd.str12(0,  30, "buf_data_x={:f}".format(buf_data_x), 0xF800)
    lcd.str12(0,  40, "buf_data_y={:f}".format(buf_data_y), 0xF800)
    lcd.str12(0,  50, f"{state}    ", 0xF800)
    lcd.str12(0,  60, "tof={:f}".format(tof_data), 0xF800)
    lcd.str12(0,  70, f"{adc_avg}", 0xF800)
    lcd.str12(0,  80, f"first={first_ready_flag}", 0xF800)
    lcd.str12(0,  90, f"second={second_ready_flag}", 0xF800)
    lcd.str12(0,  100, f"obj={obj}", 0xF800)
    lcd.str12(0,  110, f"omega={omega}", 0xF800)
    lcd.str12(0,  120, f"gx_f={gx_f}", 0xF800)
    lcd.str12(0,  130, f"x_save={X_save}", 0xF800)
    lcd.str12(0,  140, f"yellow={yellow_detect_flag}", 0xF800)
    #print(f"{eulerAngle.yaw}")
    
    if state == state_index["stop"] or state == state_index["push_2nd"] :
        buf_wireless_len = wireless.any()
        if(buf_wireless_len):
            buf_wireless = wireless.read(buf_wireless_len)
            if '1' in buf_wireless:
                first_ready_flag = 1
            
                
    
        
    #状态判断及转换函数
    if state == state_index["speed_up"]:
        if speed_up_count >= 0.4:
            state = state_index["move"]
            speed_up_count = 0
            
    
            
    elif state == state_index["move"]:
        if point_get_flag == 1:
            
            pid_ur.inlast = 0
            pid_ur.integral = 0
            pid_ul.inlast = 0
            pid_ul.integral = 0
            pid_md.inlast = 0
            pid_md.integral = 0
            state = state_index["detect"]
#             for i in range(10):
#                 uart6.write('1')
            point_get_flag = 0
            
   
       
    elif state == state_index["detect"] or state == state_index["back"]:
        buf_len = uart6.any()
        if(buf_len):
            buf = uart6.read(5)
            if buf_len == 5:
                
                buf_data_x = int(buf[0:3]) - 100
                
                
                
                       
                        
        if detect_flag >= 55:
             
             state = state_index["spin"]
             omega = 90
             pid_tof.inlast = 0
             
             #wireless.write("2")
             #uart6.write('2')
             detect_flag = 0
             find_flag = 0
             buf_data_x = 899
             buf_data_y = 899
             detect_count = 0
             detect_x_filter = 160
             obj_detect = [0,0,0]
             obj_detect_flag = 0 
             
    
    elif state == state_index["detect_2nd"]:
        buf_len = uart6.any()
        if(buf_len):
            buf = uart6.read(5)
            if buf_len == 5:
                buf_data_x = int(buf[0:3]) - 100
                
                
                if obj_detect_flag == 0 and buf_data_x != 899:
                    buf_int = int(buf[3]) - 48
                    for i in range(3):
                        if i != buf_int:
                            obj_detect[i] = 0
                    obj_detect[buf_int] += 1
                    if obj_detect[buf_int] == 3:
                        obj = buf_int
                        obj_detect_flag = 1
                
        if detect_flag >= 55 and obj != obj_index["null"]:
            #uart6.write('2')
            if obj == obj_index["ball"]:
                state = state_index["stop"]
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
                wireless.write(f"s{int(X)+100}{int(Y)+100}{obj}v")
            else:
                state = state_index["push"]
            pid_tof.inlast = 0
            #uart6.write("1")
            detect_flag = 0
            buf_data_x = 899
            buf_data_y = 899
            detect_count = 0
            detect_x_filter = 160
            obj_detect = [0,0,0]
            obj_detect_flag = 0 
  
             
   
    elif state == state_index["detect_3rd"]:
        buf_len = uart6.any()
        if(buf_len):
           
            buf = uart6.read(5)
            if buf_len == 5:
                buf_data_x = int(buf[0:3]) - 100
                
        if detect_flag >= 30:
            state = state_index["stop"]
            
            pid_tof.inlast = 0
            detect_flag = 0
            buf_data_x = 899
            buf_data_y = 899
            detect_count = 0
            detect_3rd_cnt = 0
            detect_x_filter = 160
  
    
    elif state == state_index["push_2nd"] or state == state_index["yellow_detect"] or state == state_index["home_detect"]:
        buf_len = uart6.any()
        if buf_len and not yellow_detect_flag:
            buf = uart6.read(5)
            if buf_len == 5:
                
                buf_data_x = int(buf[0:3]) - 100
                
                color_detect = int(buf[4])
                if color_detect == 1 or color_detect == 49:
                    yellow_detect_flag = 1
                    led.value(False)
                    if obj == obj_index["cube"]:
                        X_save = X
                        X = 15
                    elif obj == obj_index["ball"]:
                        Y = 270
                        
                    elif obj == obj_index["bear"]:
                        X = 305
                        
                
    
   
    if switch2.value() != state2:
        print("Test program stop.")
        break

    # 回收内存
    gc.collect()
    
















