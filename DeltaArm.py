import PCA9685

class DeltaArm:
    def __init__(self, c1, c2, c3):
        self.pwm = PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.channels = [c1,c2,c3]
        self.positions = [-1,-1,-1]
        self.max_target = 700
        self.min_target = 300

    def set_single_pwm(self,num,val):
        if val > self.max_target:
            val = max_target
        elif val < self.min_target:
            val = min_target
        self.pwm.set_pwm(self.channels[num],0,val)
        self.positions[num] = val

    def set_all_to_same_pwm(self,val):
        for i in range(3):
            self.set_single_pwm(i,val)

    def set_all_to_different_pwm(self,vals):
        for i in range(3):
            self.set_single_pwm(self,vals[i])

    def get_position(self,num):
        return self.positions[num]

    

