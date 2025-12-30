import math

class IKCalc:
    def __init__(self, x_goal, y_goal, z_goal) -> None:
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.z_goal = z_goal
        
        # link lengths / offsets
        self.a1 = 25.5 #d2z
        self.a2 = 100 #L5
        self.a3 = 100.21 #L3
        self.d1 = -50.7 #L1

        self.psy = math.radians(47.5)

        self.L4 = 50.42
        self.L5 = 100
        self.L6 = 134.16
        self.L7 = 70.94
        self.L8 = 52.5
        self.L9 = 60
        self.L10 = 24.5

        self.d2x = 19.3 
        self.d2y = 25.5 #d2z
        self.d2 = math.hypot(self.d2x, self.d2y)

        self.x_prime = 0.0
        self.z_prime = 0.0
        self.R = 0.0
        
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
    
    def wrap(self, angle : float) -> float:
        if abs(angle) > math.pi:
            return angle + 2*math.pi 
        return angle
    def wrap_neg(self, angle : float) -> float:
        # if angle < 0:
        #     return angle + 2*math.pi 
        return angle

    def calc_theta1(self, mode: bool) -> float:
        """Solve theta1; mode selects +/- sqrt branch."""
        phi = math.atan2(self.y_goal, self.x_goal)

        inner = (self.x_goal*self.x_goal) + (self.y_goal*self.y_goal) - (self.d1*self.d1)
        root = math.sqrt(inner) 

        if mode:  # positive branch
            self.theta1 = phi - math.atan2(self.d1, root)
        else:
            self.theta1 = phi - math.atan2(self.d1, -root)

        if self.x_goal < 0:
            self.theta1 = self.wrap(self.theta1)
        
        return self.theta1

    def calc_theta3(self, mode: bool) -> float:
        """Solve theta3 given theta1; mode picks elbow-up/down."""
        self.x_prime = (math.cos(self.theta1) * self.x_goal +
                        math.sin(self.theta1) * self.y_goal)
        self.z_prime = self.z_goal
        self.R = self.x_prime - self.a1

        num = (self.R*self.R +
               self.z_prime*self.z_prime -
               self.a2*self.a2 -
               self.a3*self.a3)
        den = 2.0 * self.a2 * self.a3
        costheta3 = num / den
        costheta3 = max(min(costheta3, 1.0), -1.0)  # clamp

        if mode:
            self.theta3 = math.acos(costheta3)
        else:
            self.theta3 = -math.acos(costheta3)
        return self.theta3

    def calc_theta2(self) -> float:
        """Solve theta2 using planar 2R geometry in x'-z' plane."""
        k1 = self.a2 + self.a3 * math.cos(self.theta3)
        k2 = self.a3 * math.sin(self.theta3)

        phi = math.atan2(k2, k1)
        psi = math.atan2(self.z_prime, self.R)

        self.theta2 = psi - phi

        if(self.theta2<0):
            self.theta2 = self.theta2 + 2*math.pi

        return self.theta2
    
    def solve(self, mode1: bool, mode2: bool):
        """Run full IK and return (t1, t2, t3)."""
        t1 = self.wrap_neg(self.calc_theta1(mode1))
        t3 = self.wrap_neg(self.calc_theta3(mode2))
        t2 = self.wrap_neg(self.calc_theta2())
        return t1, t2, t3
    
    def calculate(self, mode1: bool, mode2: bool) -> None:
        t1 = self.wrap_neg(self.calc_theta1(mode1))
        t3 = self.wrap_neg(self.calc_theta3(mode2))
        t2 = self.wrap_neg(self.calc_theta2())

        print("-- Results of IK --")
        print(f"mode1 (theta1 branch): {'pos' if mode1 else 'neg'}")
        print(f"mode2 (elbow branch) : {'elbow-up' if mode2 else 'elbow-down'}")
        print(f"Theta1: {math.degrees(t1):8.3f} deg")
        print(f"Theta2: {math.degrees(t2):8.3f} deg")
        print(f"Theta3: {math.degrees(t3):8.3f} deg")

    def _safe_acos(self, x: float, eps: float = 1e-9) -> float:
        """acos with clamping and domain check."""
        # if x > 1.0 + eps or x < -1.0 - eps:
        #     # truly impossible geometry
        #     raise ValueError(f"acos domain error: x={x}")
        x = max(min(x, 1.0), -1.0)
        return math.acos(x)
    
    def theta1_prime(self, theta1_mode: bool) -> float:
        if not theta1_mode:
            theta1_p = self.theta1 + math.pi/2
            return theta1_p
        return 0
    def theta2_prime(self, theta2_mode: bool) -> float:
        if not theta2_mode:
            theta2_p = self.theta2 - math.pi/2
            return theta2_p
        return 0
    def theta3_prime(self, theta1_mode: bool, theta2_mode: bool) -> float:
        if not theta1_mode and not theta2_mode:
            phi1 = math.asin(self.d2y/self.d2)
            phi2 = math.asin(self.d2x/self.d2)

            r = math.sqrt(self.L4*self.L4 + self.L5*self.L5 - 2*self.L4*self.L5*math.cos(abs(self.theta3)))
            theta_r3 = math.asin((self.L4/r) * math.sin(abs(self.theta3)))
            cos_theta_r2 = (self.L8*self.L8 + r*r - self.L6*self.L6) / (2*self.L8*r)
            theta_r2 = self._safe_acos(cos_theta_r2)


            psy_2 = math.pi - 2*self.psy

            theta_l2 = (2*math.pi - self.theta2) - theta_r3 - theta_r2
            theta_l1 = psy_2 - theta_l2

            theta_r4 = math.pi - theta_l1 - phi2
            r2 = math.sqrt(self.L8*self.L8 + self.d2*self.d2 - 2*self.L8*self.d2*math.cos(abs(theta_r4)))

            phi3 = math.asin((self.L8/r2)*math.sin(theta_r4))
            cos_ph4 = (self.L10*self.L10 + r2*r2 - self.L9*self.L9) / (2*self.L10*r2)
            phi4 = self._safe_acos(cos_ph4)


            theta2_p = 2*math.pi - abs(phi1 + phi3 + phi4)
            theta2_p = (math.pi - abs(self.theta3)) - ((2*math.pi - abs(self.theta2)) - math.pi/2) - math.radians(20) + math.pi/2
            return theta2_p
        return 0

if __name__ == "__main__":
    ik = IKCalc(-50,50.7,0)
    ik.calculate(True,True)
    ik.calculate(True,False)
    ik.calculate(False,True)
    ik.calculate(False,False)

    print(math.degrees(math.pi - ik.theta3_prime(False, False)))