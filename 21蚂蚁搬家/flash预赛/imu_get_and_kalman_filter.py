import math
from machine import *
import gc
import time
import array







def kalman_filter_explicit(x, p, z, F, H, Q, R):
    """
    直接计算卡尔曼滤波，优化计算效率
    x: [x1, x2]  当前状态估计
    p: [[p11, p12], [p21, p22]]  当前误差协方差
    z: [z1, z2]  测量值
    F, H, Q, R:  卡尔曼滤波参数矩阵
    """
    
    # 提前计算 F 的平方和乘积，减少重复运算
    F00_2 = F[0][0] * F[0][0]
    F01_2 = F[0][1] * F[0][1]
    F10_2 = F[1][0] * F[1][0]
    F11_2 = F[1][1] * F[1][1]
    F00_F01 = 2 * F[0][0] * F[0][1]
    F10_F11 = 2 * F[1][0] * F[1][1]

    # **预测步骤**
    x1_pred = F[0][0] * x[0] + F[0][1] * x[1]
    x2_pred = F[1][0] * x[0] + F[1][1] * x[1]

    # **预测 P_pred**
    p11_pred = F00_2 * p[0][0] + F00_F01 * p[1][0] + F01_2 * p[1][1] + Q[0][0]
    p12_pred = F[0][0] * F[1][0] * p[0][0] + (F[0][0] * F[1][1] + F[0][1] * F[1][0]) * p[1][0] + F[0][1] * F[1][1] * p[1][1] + Q[0][1]
    p21_pred = p12_pred  # 由于 P 是对称矩阵，p12_pred 和 p21_pred 相同
    p22_pred = F10_2 * p[0][0] + F10_F11 * p[1][0] + F11_2 * p[1][1] + Q[1][1]

    # **计算 S = H * P_pred * H^T + R**
    S11 = p11_pred + R[0][0]
    S12 = p12_pred + R[0][1]
    S21 = S12  # 对称矩阵
    S22 = p22_pred + R[1][1]

    # **计算 S 的逆**
    det_S = S11 * S22 - S12 * S21
    inv_det_S = 1 / det_S  # 直接计算倒数，减少除法操作
    S11_inv = S22 * inv_det_S
    S12_inv = -S12 * inv_det_S
    S21_inv = S12_inv  # 对称矩阵
    S22_inv = S11 * inv_det_S

    # **计算卡尔曼增益 K**
    K11 = p11_pred * S11_inv + p12_pred * S21_inv
    K12 = p11_pred * S12_inv + p12_pred * S22_inv
    K21 = p21_pred * S11_inv + p22_pred * S21_inv
    K22 = p21_pred * S12_inv + p22_pred * S22_inv

    # **计算测量残差 y = z - H * x_pred**
    y1 = z[0] - x1_pred
    y2 = z[1] - x2_pred

    # **更新状态估计**
    x1_est = x1_pred + K11 * y1 + K12 * y2
    x2_est = x2_pred + K21 * y1 + K22 * y2

    # **更新协方差 P_est**
    P11_est = (1 - K11) * p11_pred - K12 * p21_pred
    P12_est = (1 - K11) * p12_pred - K12 * p22_pred
    P21_est = -K21 * p11_pred + (1 - K22) * p21_pred
    P22_est = -K21 * p12_pred + (1 - K22) * p22_pred

    return [x1_est, x2_est], [[P11_est, P12_est], [P21_est, P22_est]]


dt_kf = 0.004   # 4ms采样间隔

# 状态转移矩阵 F = [[1, dt], [0, 1]]
F = [[1.0, dt_kf], [0.0, 1.0]]
# 观测矩阵 H = [[1, 0], [0, 1]]
H = [[1.0, 0.0], [0.0, 1.0]]
# 过程噪声协方差 Q（需调参）
Q = [[0.001, 0.0], [0.0, 0.01]]
# 观测噪声协方差 R：位置噪声小（编码器），速度噪声大（加速度积分）
R = [[0.0001, 0.0], [0.0, 1.0]]

# X方向滤波状态
x_x = [0.0, 0.0]      # [位置, 速度]
p_x = [[500.0, 0.0], [0.0, 500.0]]

# Y方向滤波状态
x_y = [0.0, 0.0]
p_y = [[500.0, 0.0], [0.0, 500.0]]


# 加速度积分全局状态
acc_integral_state = {
    'last_vx': 0.0,
    'last_vy': 0.0,
    'last_x': 0.0,
    'last_y': 0.0,
    'last_ax': 0.0,
    'last_ay': 0.0,
}

def sensor_to_car(ax_s, ay_s, az_s):
    """
    将传感器加速度转换到小车坐标系（X右，Y前，Z上）
    """
    cos30 = 0.8660254037844386
    cos60 = 0.5
    
    ax_c = cos60 * ax_s - cos30 * ay_s
    ay_c = cos30 * ax_s + cos60 * ay_s
    az_c = -az_s
    return ax_c, ay_c, az_c

def accel_integral(ax, ay, az, yaw_deg, dt, reset=False):
    
    """
    使用加速度计数据积分计算位移（重力补偿 + 梯形积分）
    参数：
        ax, ay, az: 原始加速度计读数（单位：g 或 m/s²，需统一）
        roll_deg, pitch_deg: 当前姿态角（度）
        dt: 采样间隔（秒）
        reset: 是否重置积分状态（初始化为零）
    返回：
        (dx, dy) 本次时间间隔内的位移增量（米）
    """
    global acc_integral_state

    if reset:
        acc_integral_state = {
            'last_vx': 0.0,
            'last_vy': 0.0,
            'last_x': 0.0,
            'last_y': 0.0,
            'last_ax': 0.0,
            'last_ay': 0.0,
        }
        return 0.0, 0.0

    # 转换为米/秒²（假设输入已经是 m/s²，如果不是请转换）
    # 此处假设输入已经是 m/s²，如果传感器输出是 g，需乘以 9.8
    # 例如：ax_mps2 = ax * 9.80665

    # 重力补偿：将加速度从机体坐标系转换到水平坐标系
    # 旋转矩阵：先绕 x 轴旋转 roll，再绕 y 轴旋转 pitch
    yaw = math.radians(yaw_deg)

    # 转换到水平面（X东，Y北，Z天）
    ax_h = ax * math.cos(yaw - math.pi/2) + ay * math.cos(yaw)
    ay_h = ax * math.sin(yaw - math.pi/2) + ay * math.sin(yaw)
   
    # 水平方向的加速度即为 x, y 方向
    acc_x_h = ax_h
    acc_y_h = ay_h

    # 梯形积分更新速度
    vx = acc_integral_state['last_vx'] + (acc_x_h + acc_integral_state['last_ax']) / 2.0 * dt
    vy = acc_integral_state['last_vy'] + (acc_y_h + acc_integral_state['last_ay']) / 2.0 * dt

    # 更新位置
    x = acc_integral_state['last_x'] + (vx + acc_integral_state['last_vx']) / 2.0 * dt
    y = acc_integral_state['last_y'] + (vy + acc_integral_state['last_vy']) / 2.0 * dt
    
   
    
    # 计算本次增量
    dx = x - acc_integral_state['last_x']
    dy = y - acc_integral_state['last_y']

    # 保存状态用于下次积分
    acc_integral_state['last_vx'] = vx
    acc_integral_state['last_vy'] = vy
    acc_integral_state['last_x'] = x
    acc_integral_state['last_y'] = y
    acc_integral_state['last_ax'] = acc_x_h
    acc_integral_state['last_ay'] = acc_y_h

    return dx, dy



