import PCA9685


class DeltaArm:
    def __init__(self, c1, c2, c3):
        self.pwm = PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.channels = [c1,c2,c3]
        self.positions = [-1,-1,-1]
        self.angles = [-1, -1, -1]
        self.zero_vals = [370, 335, 325]
        self.ninety_vals = [640, 585 ,595]
        #inches
        self.fixed_edge = 4
        self.effector_edge = 1.5
        self.upper_len = 5
        self.lower_len = 7

    def set_single_pwm(self,num,val):
        self.pwm.set_pwm(self.channels[num],0,val)
        self.positions[num] = val
        self.angles[num] = self.position_to_angle(num, val)

    def set_all_to_same_pwm(self,val):
        for i in range(3):
            self.set_single_pwm(i,val)

    def set_all_to_different_pwm(self,vals):
        for i in range(3):
            self.set_single_pwm(self,vals[i])

    def set_single_angle(self,num,ang):
        val = int(self.angle_to_position(num, ang))
        self.pwm.set_pwm(self.channels[num],0,val)
        self.positions[num] = val
        self.angles[num] = ang
    
    def set_all_to_same_angle(self,ang):
        for i in range(3):
            self.set_single_angle(i,ang)

    def set_all_to_different_angle(self,angs):
        for i in range(3):
            self.set_single_angle(self,angss[i])   
            
    def get_position(self,num):
        self.angles[num] = self.position_to_angle(val)    
        return self.positions[num]

    def get_angle(self, num):
        return self.angles[num]

    def position_to_angle(self,num,pos):
        return (pos - self.zero_vals[num])*90.0/(self.ninety_vals[num] - self.zero_vals[num])
    
    def angle_to_position(self,num, ang):
        return (ang)*(self.ninety_vals[num] - self.zero_vals[num])/90.0 + self.zero_vals[num]



    

