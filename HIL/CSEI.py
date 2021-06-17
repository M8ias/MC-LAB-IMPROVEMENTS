import math
import numpy as np

from Kinematics import Rzyx


# The C/S enterprise I object
class CSEI:
    """
    The CSEI object represents the C/S Enterprise and contains the necessary
    kinematics and dynamics of the ship, as well as the operations required to
    "move" the ship over one time-step
    """
    # Class attributes
    __M = np.array([[16.11, 0.0, 0.0], [0.0, 24.11, 0.5291], [0.0, 0.5291, 2.7600]])  # Inertia matrix
    _X = np.array([[-0.655, 0.3545, -3.787], [0.0, -2.443, 0], [0.0, 0.0, 0.0]])  # Damping coefficients in surge
    _Y = np.array([[-1.33, -2.776, -64.91], [-2.8, -3.450, 0], [-0.805, -0.845, 0]])  # Damping coeffcients in sway
    _Z = np.array([[0, -0.2088, 0], [-1.90, -0.75, 0], [0.130, 0.080, 0]])  # Damping coeffcients in yaw
    _A = np.array([[-2.0, 0.0, 0.0], [0.0, -10.0, 0.0], [0.0, 0.0, -1.0]])  # Added mass matrix
    _m = 14.11  # Rigid body mass
    _Iz = 1.7600  # Inertial moment
    _xg = 0.0375  # Center of gravity

    _L = np.array([[-0.4574, -0.4574, 0.3875], [-0.055, 0.055, 0]])  # Placement of actuators
    _K = np.array([[1.03, 0, 0], [0, 1.03, 0], [0, 0, 2.629]])  # K-matrix

    def __init__(self, eta0):
        self.D = np.array([np.zeros(3), np.zeros(3), np.zeros(3)])
        self.C = np.array([np.zeros(3), np.zeros(3), np.zeros(3)])
        self.eta = eta0
        self.nu = np.array([[0], [0], [0]])  # Assuming the vessel always starts stationary
        self.tau = np.array([[0], [0], [0]])  # Zero forces and moments at initialization
        self.nu_dot = np.array([[0], [0], [0]])
        self.eta_dot = np.array([[0], [0], [0]])

    def set_D(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        d11 = -self._X[0][0] - self._X[0][1]*np.abs(u) - self._X[0][2]*(u**2)
        d22 = -self._Y[0][0] - self._Y[0][1]*np.abs(v) - self._Y[0][2]*(v**2) - self._Y[2][0]*np.abs(r)
        d33 = -self._Z[1][0] - self._Z[1][1]*np.abs(r) - self._Z[1][2]*(r**2) - self._Z[2][1]*np.abs(v)
        d23 = -self._Y[1][0] - self._Y[1][1]*np.abs(r) - self._Y[1][2]*(r**2) - self._Y[2][1]*np.abs(v)
        d32 = -self._Z[0][0] - self._Z[0][1]*np.abs(v) - self._Z[0][2]*(v**2) - self._Z[2][0]*np.abs(r)
        d11 = d11[0]  # This is pretty scuffed and should be optimized. Just need to fin out why it returns a 1x1 vector
        d22 = d22[0]
        d33 = d33[0]
        d23 = d23[0]
        d32 = d32[0]
        new_D = np.array([[d11, 0, 0], [0, d22, d23], [0, d32, d33]])
        self.D = new_D  # Designates the damping matrix

    def set_C(self):
        u = self.nu[0]
        v = self.nu[1]
        r = self.nu[2]
        c13 = -(self._m - self._A[1][1])*v - (self._m*self._xg - self._A[2][1])*r
        c23 = (self._m - self._A[0][0])*u
        c13 = c13[0]  # See comment about scuffedness above
        c23 = c23[0]
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

    def set_eta(self, dt):
        psi = self.eta[2]
        R = Rzyx(psi)
        self._eta_dot = np.dot(R, self.nu)
        self.eta = self.eta + dt*self._eta_dot

    def set_nu(self, dt):
        nom = self.__M
        den = self.tau - np.dot((self.C + self.D), self.nu)
        nom_inv = np.linalg.inv(nom)
        self.nu_dot = np.dot(nom_inv, den)
        self.nu = self.nu + dt*self.nu_dot  # Integration

    def get_tau(self):
        return self.tau

    def get_eta(self):
        return self.eta

    def get_nu(self):
        return self.nu

