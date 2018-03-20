import Slush
import math
import time

class DeltaArm:
    def __init__(self, c1, c2, c3):
        self.board = Slush.sBoard()
        self.motors = [Slush.Motor(c1),Slush.Motor(c2),Slush.Motor(c3)]
        for m in self.motors:
            m.setCurrent(20,100,100,100)
            m.setAccel(750)
            m.setMaxSpeed(750)
        self.positions = [-1,-1,-1]
        self.angles = [-1, -1, -1]
        self.zero_vals = [-2000, -2000, -2000]
        self.ninety_vals = [64000, 64000 ,64000]
        #angles of each effector arm relative to coordinate axis
        self.phi_vals = [math.radians(210), math.radians(90), math.radians(330)]
        #feet
        self.fixed_edge = .8828
        self.effector_edge = .28867
        self.upper_len = 1.438
        self.lower_len = 2.90188
        self.end_effector_z_offset = .083

    def home_all(self):
        for m in self.motors:
            while m.isBusy():
                continue
            m.goUntilPress(0,0,5000)
         


    def set_single_position_steps(self,num,pos):
        self.positions[num] = pos
        self.angles[num] = self.position_to_angle(num, pos)
        while self.motors[num].isBusy():
            continue
        self.motors[num].goTo(pos)
        print(pos)

    def set_all_to_same_position(self,val): 
        for i in range(3):
            self.set_single_position_steps(i,val)
            time.sleep(.1)


    def set_single_angle(self,num,ang):
        val = int(self.angle_to_position(num, ang))
        self.set_single_position_steps(num,val)
        self.positions[num] = val
        self.angles[num] = ang
    
    def set_all_to_same_angle(self,ang):
        for i in range(3):
            self.set_single_angle(i,ang)

    def set_all_to_different_angle(self,a1,a2,a3):
        angs = [a1,a2,a3]
        for i in range(3):
            self.set_single_angle(i,angs[i]) 

    def stop_all(self):
        self.motors[0].hardStop()
        self.motors[1].hardStop()
        self.motors[2].hardStop()

    
    def reset_pos_all(self):
        self.motors[0].setAsHome()
        self.motors[1].setAsHome()
        self.motors[2].setAsHome()

    def get_position(self,num):
        self.angles[num] = self.position_to_angle(self.positions[num])    
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
        z0 = z0 + self.end_effector_z_offset

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
        




     

