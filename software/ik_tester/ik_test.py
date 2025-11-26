import math

class IKCalc:
    def __init__(self, x_goal, y_goal, z_goal) -> None:
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.z_goal = z_goal
        
        # link lengths / offsets
        self.a1 = 25.5
        self.a2 = 100
        self.a3 = 100.21
        self.d1 = 50.7  # renamed from y

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
        return self.theta2
    
    def solve(self, mode1: bool, mode2: bool):
        """Run full IK and return (t1, t2, t3)."""
        t1 = self.calc_theta1(mode1)
        t3 = self.calc_theta3(mode2)
        t2 = self.calc_theta2()
        return t1, t2, t3
    
    def calculate(self, mode1: bool, mode2: bool) -> None:
        t1 = self.calc_theta1(mode1)
        t3 = self.calc_theta3(mode2)
        t2 = self.calc_theta2()

        print("-- Results of IK --")
        print(f"mode1 (theta1 branch): {'pos' if mode1 else 'neg'}")
        print(f"mode2 (elbow branch) : {'elbow-up' if mode2 else 'elbow-down'}")
        print(f"Theta1: {math.degrees(t1):8.3f} deg")
        print(f"Theta2: {math.degrees(t2):8.3f} deg")
        print(f"Theta3: {math.degrees(t3):8.3f} deg")


if __name__ == "__main__":
    ik = IKCalc(-10,50.7,0)
    ik.calculate(True,True)
    ik.calculate(True,False)
    ik.calculate(False,True)
    ik.calculate(False,False)