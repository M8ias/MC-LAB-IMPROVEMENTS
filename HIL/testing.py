import math
import numpy as np

from Kinematics import Rzyx
from CSEI import CSEI



initial_condition = np.array([[0] , [0], [math.pi/4]])
ship = CSEI(initial_condition)
u = np.array([0.5, 0.5, 0, -math.pi/2, -math.pi/2])
dt = 0.01

t = np.linspace(0, 100, 10000)
ticker = 0

while ticker < t[-1]: 
    ship.set_C()
    ship.set_D()        
    ship.set_tau(u)
    ship.set_nu(dt)
    ship.set_eta(dt)
    print(ship.get_eta())
    ticker += 0.01