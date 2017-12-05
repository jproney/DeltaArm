import PCA9685
import math

class DeltaArm:
    def __init__(self, c1, c2, c3):
        self.pwm = PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)
        self.channels = [c1,c2,c3]
        self.positions = [-1,-1,-1]
        self.angles = [-1, -1, -1]
        self.zero_vals = [370, 335, 325]
        self.ninety_vals = [640, 585 ,595]
        #angles of each effector arm relative to coordinate axis
        self.phi_vals = [math.radians(240+45), math.radians(45), math.radians(120+45)]
        #inches
        self.fixed_edge = 4.0
        self.effector_edge = 1.5
        self.upper_len = 5.0
        self.lower_len = 7.0

    def set_single_pwm(self,num,val):
        self.pwm.set_pwm(self.channels[num],0,val)
        self.positions[num] = val
        self.angles[num] = self.position_to_angle(num, val)

    def set_all_to_same_pwm(self,val):
        for i in range(3):
            self.set_single_pwm(i,val)

    def set_all_to_different_pwm(self,v1,v2,v3):
        vals = [v1,v2,v3]
        for i in range(3):
            self.set_single_pwm(vals[i])

    def set_single_angle(self,num,ang):
        val = int(self.angle_to_position(num, ang))
        self.pwm.set_pwm(self.channels[num],0,val)
        self.positions[num] = val
        self.angles[num] = ang
    
    def set_all_to_same_angle(self,ang):
        for i in range(3):
            self.set_single_angle(i,ang)

    def set_all_to_different_angle(self,a1,a2,a3):
        angs = [a1,a2,a3]
        for i in range(3):
            self.set_single_angle(i,angs[i])   
            
    def get_position(self,num):
        self.angles[num] = self.position_to_angle(val)    
        return self.positions[num]

    def get_angle(self, num):
        return self.angles[num]

    def position_to_angle(self,num,pos):
        return (pos - self.zero_vals[num])*90.0/(self.ninety_vals[num] - self.zero_vals[num])
    
    def angle_to_position(self,num, ang):
        return (ang)*(self.ninety_vals[num] - self.zero_vals[num])/90.0 + self.zero_vals[num]

    @staticmethod
    def wrap_angle_rad(theta):
        while abs(theta) > math.pi:
            if theta < math.pi:
                theta += 2*math.pi
            if theta > math.pi:
                theta -= 2*math.pi
        return theta

    def rotate_point_to_yz_plane(self,x0,y0,z0,phi):
        #do rotation matrix
        x = x0*math.cos(phi) + y0*math.sin(phi)
        y = -x0*math.sin(phi) + y0*math.cos(phi)

        #z is the same
        z = z0
        return (x,y,z)


    def inverse_kinematics_in_yz_plane(self,x0,y0,z0):
        # parameters
        rf = self.upper_len
        re = self.lower_len
        f = self.fixed_edge
        e = self.effector_edge

        #linear coefficients of EQN z = b*y + a

        a = (x0**2 + (y0-e/(2*math.sqrt(3)))**2 + z0**2 + rf**2 - re**2 - f**2/12)/(2*z0) 
        b = (-f/(2*math.sqrt(3)) - y0 + e/(2*math.sqrt(3)))/z0

        #plug line (z = b*y + a) into circle in yz w/ center (-f/2sqrt(3),0)

        disc = (f/math.sqrt(3) + 2*a*b) - 4*(b**2+1)*(f**2/12 + a**2 - rf**2)
        if disc < 0:
            #disciminate < 0 -> no solution
            return -1

        #compute solution w/ lower y value
        y = (-(f/math.sqrt(3) + 2*a*b) - math.sqrt(disc))/(2*(b**2+1))
        z = b*y + a

        theta = DeltaArm.wrap_angle_rad(math.atan(z/(y + f/(2*math.sqrt(3)))))
        return math.degrees(theta)
        
    def compute_triple_inverse_kinematics(self, x, y, z):
        thetas = []
        for phi in self.phi_vals:
            (x0,y0,z0) = self.rotate_point_to_yz_plane(x,y,z,phi)
            theta = self.inverse_kinematics_in_yz_plane(x0,y0,z0)
            if theta == -1:
                raise ValueError('that point is impossible!')
            thetas.append(theta)
        return (thetas[0], thetas[1], thetas[2])

    def move_to_point(self,x,y,z):
        (a1,a2,a3) = self.compute_triple_inverse_kinematics(x,y,z)
        self.set_all_to_different_angle(a1,a2,a3)





     

