import gc
import time
import math
import array
from move_ctrl import *
SAMPLING_INTERVAL_MS = 4  # 采样间隔5ms
ENCODER_PULSES_PER_REVOLUTION =  575 # 每圈脉冲数

#小车物理参数
distance = 1.0  #每间隔1cm记录一次点位
X = 46.0
Y = -20.0
#每走过1cm,NUM+1
NUM_Plus = 0


#x轴数据
x_total_pulses = 0
x_last_total_pulses = 0
x_encoder_last = 0
x_encoder_prev_speed = 0
target_pulses_y = 27
target_pulses_x = 27
#y轴数据
y_total_pulses = 0
y_last_total_pulses = 0
y_encoder_last = 0
y_encoder_prev_speed = 0
        

'''
def update_wheel_pulses(x_current_encoder, y_current_encoder):
    global x_total_pulses,x_last_total_pulses,x_encoder_last,x_encoder_prev_speed,target_pulses,y_total_pulses,y_last_total_pulses,y_encoder_last,y_encoder_prev_speed
    x_delta = x_current_encoder
    if x_delta >= 500:
        x_delta = 0
    x_forward_crossings = 0 #在函数开始时声明并初始化
    y_forward_crossings = 0
    x_total_pulses += x_current_encoder   
    y_delta = y_current_encoder
    if y_delta >= 500:
        y_delta = 0
    y_total_pulses += y_current_encoder
    return x_forward_crossings, y_forward_crossings, x_total_pulses, y_total_pulses #4. 返回本次计算出的前进厘米数
'''

# 用于加速度重力补偿的姿态角
global_roll = 0.0
global_pitch = 0.0
position_x_enc = 0
position_y_enc = 0
last_acc_x = 0
last_acc_y = 0
last_v_x = 0
last_v_y = 0
last_position_x_acc = 0
last_position_y_acc = 0
position_x_enc= 0
position_y_enc = 0
last_fused_x = 0
last_fused_y = 0
dt_acc = 0.004

#脉冲累计
def update_wheel_pulses(x_current_encoder, y_current_encoder,pulse_x,pulse_y):
    global x_total_pulses,x_last_total_pulses,x_encoder_last,x_encoder_prev_speed,y_total_pulses,y_last_total_pulses,y_encoder_last,y_encoder_prev_speed
    x_delta = x_current_encoder
    y_delta = y_current_encoder
    target_pul_x = pulse_x
    target_pul_y = pulse_y
    x_forward_crossings = 0 #在函数开始时声明并初始化
    y_forward_crossings = 0 #在函数开始时声明并初始化
    if abs(x_delta) > 1000:
        x_total_pulses = 0 #重置累计脉冲，避免跳变干扰后续计数 #本次调用不计算位移，因为发生了跳变重置
        x_forward_crossings = 0 #确保跳变时返回0
    else: 
        #正常脉冲累加和计数值计算
        x_forward_crossings = x_delta / target_pul_x
    if abs(y_delta) > 1000:
        y_total_pulses = 0 #重置累计脉冲，避免跳变干扰后续计数 #本次调用不计算位移，因为发生了跳变重置
        y_forward_crossings = 0 #确保跳变时返回0
    else: 
        #正常脉冲累加和计数值计算
        y_forward_crossings = y_delta / target_pul_y
    
    return x_forward_crossings, y_forward_crossings  #4. 返回本次计算出的前进厘米数


def update_wheel_pulses_acc(acc_x,acc_y):
    global last_acc_x,last_acc_y,last_v_x,last_v_y,last_position_x_acc,last_position_y_acc
    
    # ------------------- 加速度计部分 -------------------
    # 获取原始加速度（需根据实际传感器范围转换）
    acc_x_comp = acc_x      # 单位：g 或 m/s²，需提前标定
    acc_y_comp = acc_y
    
    # 重力补偿（假设 roll, pitch 由 altitude_calc 更新到全局变量）
   
    dt = 0.004
    

    # 梯形积分更新速度
    v_x = last_v_x + (acc_x_comp + last_acc_x) / 2 * dt
    v_y = last_v_y + (acc_y_comp + last_acc_y) / 2 * dt

    # 更新位置
    position_x_acc = last_position_x_acc + (v_x + last_v_x) / 2 * dt
    position_y_acc = last_position_y_acc + (v_y + last_v_y) / 2 * dt

    # 更新状态
    last_acc_x = acc_x_comp
    last_acc_y = acc_y_comp
    last_v_x = v_x
    last_v_y = v_y
    last_position_x_acc = position_x_acc
    last_position_y_acc = position_y_acc

    # ------------------- 融合 -------------------
    # 互补滤波系数，编码器更可靠（短时）取大一些
#     alpha_enc = 0.95
#     fused_x = alpha_enc * position_x_enc + (1 - alpha_enc) * position_x_acc
#     fused_y = alpha_enc * position_y_enc + (1 - alpha_enc) * position_y_acc
# 
#     # 计算本次增量（浮点数）
#     delta_x = fused_x - last_fused_x
#     delta_y = fused_y - last_fused_y
# 
#     # 更新上次融合位置
#     last_fused_x = fused_x
#     last_fused_y = fused_y

    # 返回浮点增量，不取整
    return position_x_acc, position_y_acc


''' 
/**
 * @brief 获取轮子在固定时间窗口5ms内的速度（脉冲增量）
 *
 * @param wheel 轮子数据指针
 * @param current_encoder 当前编码器值
 * @return int32_t 时间窗口内的脉冲增量（正值为正向，负值为反向）
 *
 * @说明
 * - 此函数应在定时中断（如5ms）中调用，确保固定时间间隔。
 * - 每次调用会更新encoder_prev_speed为当前值，供下次计算使用。
 */ 
 '''

#路径记忆函数,return小车走1cm，x，y坐标分别增加量
def distance_get_plus(cross:int, thieta:int):
    global NUM_Plus
    NUM_Plus += cross
    delta_x = cross * (math.cos(thieta))
    delta_y = cross * (math.sin(thieta))
    return delta_x,delta_y
'''
y
^       。
|      /
|     /
|    /
|   /
|  /
| /
|/thieta
。------------------------>x
'''

point_get_flag = 0#到达终点标志位
#前往目标点函数,输入当前坐标和目标坐标，输出运动模式和速度
def goto_point(x_now, y_now, x_pur, y_pur):
    global point_get_flag,move_mode,move_mode_index
    global pid_point_get
    distance = 0.0
    if abs(x_pur - x_now) <= 0.01:
        if y_pur > y_now:
            pur_thieta = 90
        else:
            pur_thieta = 270
    else:
        k = (y_pur - y_now) / (x_pur - x_now)
        
        pur_thieta  = math.atan(k) * 57.295779513082320876798154814105#弧度转角度
        if x_pur < x_now:
            pur_thieta += 180
    
    distance = math.sqrt((x_pur - x_now)**2 + (y_pur - y_now)**2)
    speed = pid_point_get.compute(0, -distance)
    #speed = 80
    
    if distance <= 1:
        return 90, 0 , 1


    return pur_thieta , speed , 0

def sigmoid_speed_curve(t, v_start, v_target, T, k=6.0):
    """
    返回在时间t处的速度，使用sigmoid曲线。
    t: 当前时间（0 <= t <= T）
    v_start: 起始速度
    v_target: 目标速度
    T: 加速总时间
    k: 控制斜率，默认6使得在t=0和t=T时速度接近端点值
    """
    # 归一化时间，然后映射到sigmoid的自变量范围
    # 使用sigmoid: y = 1/(1+exp(-k*(t/T - 0.5)*?))
    # 实际上：x = k * (t/T - 0.5) 使得在t=0时x=-k/2，t=T时x=k/2
    # 当k=6时，y(0)=1/(1+exp(3))≈0.047, y(T)=1/(1+exp(-3))≈0.953，这还不够接近。
    # 因此我们使用更大的系数，比如k=12，则y(0)=1/(1+exp(6))≈0.0025，y(T)=0.9975。
    # 实际上我们想要在端点处尽可能接近0和1，但避免除以零。可以使用k=12。
    # 更好的方式：让用户指定斜率因子。
    e =  2.71828
    x = k * (t/T - 0.5)
    y = 1.0 / (1.0 + e**(-x))
    # y的范围是(0,1)，映射到速度范围
    v = v_start + (v_target - v_start) * y
    return v
'''
# 示例参数
v_start = 0.0   # 起始速度（单位取决于电机）
v_target = 100.0 # 目标速度
T = 2.0         # 加速时间（秒）
k = 12.0        # 斜率因子
'''




