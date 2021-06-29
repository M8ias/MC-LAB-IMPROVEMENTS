import numpy as np
import math

def take(alpha, u):
    K = np.array([[2.629, 0, 0],[0, 1.03, 0],[0, 0, 1.03]])
    lx1 = 0.3875
    lx2 = -0.4574
    lx3 = -0.4574
    ly1 = 0
    ly2 = -0.055
    ly3 = 0.055
    c1 = 0
    c2 = math.cos(alpha[0])
    c3 = math.cos(alpha[1])
    s1 = 1
    s2 = math.sin(alpha[0])
    s3 = math.sin(alpha[1])
    B = np.array([[c1, c2, c3], [s1, s2, s3], [lx1*s1-ly1*c1, lx2*s2 - ly2*c2, lx3*s3-ly3*c3]])
    new_tau = np.dot(np.dot(B,K), u)
    print(B)
    print(K)
    print(np.dot(B,K))
    print(new_tau)
alpha = np.array([0, 0])
u = np.array([[0],[0.9709],[0.9709]])
take(alpha, u)