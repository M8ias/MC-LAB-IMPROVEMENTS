import math
from observer.observer.math_tools import yaw2quat
import numpy as np
import rospy

from Kinematics import yaw2quat, Rzyx
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

# The C/S enterprise I object
class CSEI:
    """
    The CSEI object represents the C/S Enterprise and contains the necessary
    kinematics and dynamics of the ship, as well as the operations required to
    "move" the ship over one time-step
    """
    ### Main data of the C/S Enterprise. Do not touch ###
    __M = np.array([[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]])  # Inertia matrix
    _X = np.array([[-0.655, 0.3545, -3.787], 
                    [0.0, -2.443, 0.0], 
                    [0.0, 0.0, 0.0]])  # Damping coefficients in surge
    _Y = np.array([[-1.33, -2.776, -64.91], 
                    [-2.8, -3.450, 0], 
                    [-0.805, -0.845, 0]])  # Damping coeffcients in sway
    _Z = np.array([[0, -0.2088, 0], 
                   [-1.90, -0.75, 0], 
                   [0.130, 0.080, 0]])  # Damping coeffcients in yaw
    _A = np.array([[-2.0, 0.0, 0.0], [0.0, -10.0, 0.0], [0.0, 0.0, -1.0]])  # Added mass matrix
    _m = 14.11  # Rigid body mass
    _Iz = 1.7600  # Inertial moment
    _xg = 0.0375  # Center of gravity

    _L = np.array([[-0.4574, -0.4574, 0.3875], [-0.055, 0.055, 0]])  # Placement of actuators
    _K = np.diag(np.diag([1.03, 1.03, 2.629]))  # K-matrix

    ### Initialization ###

    def __init__(self, eta0):
        self.D = np.array([np.zeros(3), np.zeros(3), np.zeros(3)])
        self.C = np.array([np.zeros(3), np.zeros(3), np.zeros(3)])
        self.eta = eta0
        self.nu = np.array([[0], [0], [0]])  # Assuming the vessel always starts stationary
        self.tau = np.array([[0], [0], [0]])  # Zero forces and moments at initialization
        self.nu_dot = np.array([[0], [0], [0]])
        self.eta_dot = np.array([[0], [0], [0]])
        self.odom = Odometry() #Msg to be published
        self.pubOdom = rospy.Publisher('/qualisys/CSEI/odom', Odometry, queue_size=1)
        self.pubTau = rospy.Publisher('/CSEI/tau', Float64MultiArray, queue_size=1)
        self.subU = rospy.Subscriber('/CSEI/u', Float64MultiArray, callback)
        self.u = np.zeros(5)
        self.publishOdom() #Publishes the initial state to odometry
        self.publishTau() #Publishes the initial tau
        self.dt
    ### Computation ###
        
    def set_D(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        d11 = (-self._X[0][0] - self._X[0][1]*np.abs(u) - self._X[0][2]*(u**2))[0]
        d22 = (-self._Y[0][0] - self._Y[0][1]*np.abs(v) - self._Y[0][2]*(v**2) - self._Y[2][0]*np.abs(r))[0]
        d33 = (-self._Z[1][0] - self._Z[1][1]*np.abs(r) - self._Z[1][2]*(r**2) - self._Z[2][1]*np.abs(v))[0]
        d23 = (-self._Y[1][0] - self._Y[1][1]*np.abs(r) - self._Y[1][2]*(r**2) - self._Y[2][1]*np.abs(v))[0]
        d32 = (-self._Z[0][0] - self._Z[0][1]*np.abs(v) - self._Z[0][2]*(v**2) - self._Z[2][0]*np.abs(r))[0] #np.arrays are scuffed
        new_D = np.array([[d11, 0, 0], [0, d22, d23], [0, d32, d33]])
        self.D = new_D  # Designates the damping matrix

    def set_C(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        c13 = (-(self._m - self._A[1][1])*v - (self._m*self._xg - self._A[2][1])*r)[0]
        c23 = ((self._m - self._A[0][0])*u)[0]
        new_C = np.array([[0, 0, c13], [0, 0, c23], [-c13, -c23, 0]])
        self.C = new_C

    def set_tau(self, u):
        u_t = np.transpose(np.take(u, [0, 1, 2])[np.newaxis])
        alpha = np.take(u, [3, 4])
        c1 = math.cos(alpha[0])
        c2 = math.cos(alpha[1])
        c3 = 0
        s1 = math.sin(alpha[0])
        s2 = math.sin(alpha[1])
        s3 = 1
        B = np.array([[c1, c2, c3], [s1, s2, s3], [self._L[0][0]*s1 - self._L[1][0]*c1, self._L[0][1]*s1 - self._L[1][1]*c1, self._L[0][2]*s3 - self._L[1][2]*c3]])
        new_tau = np.dot(np.dot(self._K, B), u_t)
        self.tau = new_tau

    def set_eta(self):
        psi = self.eta[2]
        R = Rzyx(psi)
        self._eta_dot = np.dot(R, self.nu)
        self.eta = self.eta + self.dt*self._eta_dot

    def set_nu(self):
        nom = self.__M
        den = self.tau - np.dot((self.C + self.D), self.nu)
        M_inv = np.linalg.inv(nom) # Inverts the inertia matrix
        self.nu_dot = np.dot(M_inv, den)
        self.nu = self.nu + self.dt*self.nu_dot  # Integration, forward euler

    def get_tau(self):
        return self.tau

    def get_eta(self):
        return self.eta

    def get_nu(self):
        return self.nu

    ### Publishers and subscribers ###   

    def nav_msg(self):
        """
        Computes the Odometry message of the ship
        """
        quat = yaw2quat(self.eta[2][0])

        self.odom.pose.pose.position.x = self.eta[0]
        self.odom.pose.pose.position.y = self.eta[1]
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.w = quat[0]
        self.odom.pose.pose.orientation.x = quat[1]
        self.odom.pose.pose.orientation.y = quat[2]
        self.odom.pose.pose.orientation.z = quat[3]

        self.odom.twist.twist.linear.x = self.nu[0]
        self.odom.twist.twist.linear.y = self.nu[1]
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = self.nu[2]

    def get_odom(self):
        return self.odom

    def publishOdom(self):
        self.nav_msg()
        self.pubOdom.publish(self.odom)
    
    def publishTau(self):
        self.publishTau.publish(self.tau)

    #Upon a new U, move the ship
    def callback(self, msg):
        self.u = msg.data
        self.set_C()  # Coreolis matrix
        self.set_D()  # Compute damping matrix
        self.set_tau(self.u) # Compute the force vector
        self.publishTau()   # Publish the tau, this is needed for the Observer :)
        self.set_nu(0.01)   # Compute the velocity
        self.set_eta(0.01)  # Compute the position
        self.publishOdom() # Publish the new position


    ### End of publishers and subscribers ###


def main():
    initial_conditions = np.array([[0],[0],[math.pi/2]])
    rospy.init_node('HIL_simulation')
    ship = CSEI(initial_conditions)
    rospy.spin()
    rospy.shutdown()

if __name__ == '__main__':
    main()

