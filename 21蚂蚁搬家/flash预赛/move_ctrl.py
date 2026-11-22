import math
from machine import *







#每个轮子速度计算
def speed_get(move_speed, move_angle, turn_speed):
    global motor_md_speed,motor_ur_speed,motor_ul_speed 
    zp = 1
    z_speed = turn_speed
    if z_speed >  180:
        z_speed -= 360
    if z_speed < -180:
        z_speed += 360
    motor_md =     (math.cos(math.radians(0  + move_angle))) * move_speed + z_speed * zp
    motor_ur =    -(math.cos(math.radians(60 + move_angle))) * move_speed + z_speed * zp
    motor_ul =    -(math.cos(math.radians(60 - move_angle))) * move_speed + z_speed * zp
    return motor_md,motor_ul,motor_ur




class PIDController:
    def __init__(self , kp, ki, kd, inte_limit, out_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0 
        self.inlast = 0
        self.inte_limit = inte_limit
        self.out_limit = out_limit
        
   
    
    def compute(self, pur, data):
        in_now =  pur - data
        
        self.integral += in_now
            
        if self.integral > self.inte_limit:
            self.integral = self.inte_limit
        elif self.integral < -self.inte_limit:
            self.integral = -self.inte_limit
            
        out = self.kp * in_now + self.ki * self.integral + self.kd * (in_now - self.inlast)
        
        if out >= self.out_limit:#输出限幅，防止电机过载
            out = self.out_limit
            
        elif out <= -self.out_limit:
            out = -self.out_limit
        
        self.inlast = in_now 
        return out
    
    def compute_theita(self , pur, data ,w):
        theita = data
        if theita<-180:
            theita += 360
        elif theita>180:
            theita -=360
        in_now =  pur - theita
        
        self.integral += in_now
            
        if self.integral > self.inte_limit:
            self.integral = self.inte_limit
        elif self.integral < -self.inte_limit:
            self.integral = -self.inte_limit
            
        out = self.kp * in_now + self.ki * self.integral + self.kd * (in_now - self.inlast)
        
        if out >= self.out_limit:#输出限幅，防止电机过载
            out = self.out_limit
            
        elif out <= -self.out_limit:
            out = -self.out_limit
        
        self.inlast = in_now
        theita_out = pid_theita_w.compute(out,w)
        return theita_out
    
pid_ul = PIDController(103.2,5.8,1.45,10000,5500)
pid_ur = PIDController(103.2,5.8,1.45,10000,5500)
pid_md = PIDController(103.2,5.8,1.45,10000,5500)
pid_theita = PIDController(0.85,0,0.025,1000,250)
pid_theita_w = PIDController(0.85,0,0.025,1000,300)
pid_theita_end = PIDController(0.25,0,0.004,1000,50)
pid_point_get = PIDController(1.0,0.00,0.0002,5000,70)
pid_tof = PIDController(0.09,0,0.0,2000,100)
pid_detect = PIDController(0.168*1.4,0,0.001,2000,100)






