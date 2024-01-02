import numpy as np


class SinglePendulum:

    def __init__(self,
                 m=100e-3,  # Mass of the pendulum
                 long=100e-3,  # Length of the arm
                 kt=27.8e-3,  # Torque constant
                 km=277.6e-4,  # Back-electromotive force constant
                 rm=9.73,  # Terminal resistance
                 c=0.001  # Damping coefficient of the pendulum: 0.001
                 ):
        self.m = m
        self.long = long
        self.i = (1 / 3) * m * long ** 2  # Moment of inertia of the pendulum
        self.g = 9.81  # Magnitude of the gravity
        self.kt = kt
        self.km = km
        self.rm = rm
        self.c = c
        self.dt = 1e-2  # Constant time difference between states

    def new_state_calculation(self, tho, wo, u):

        hp = 1/(self.m * self.long ** 2 + self.i)
        val_b = self.kt/self.rm
        val_c = self.c + (self.km * self.kt / self.rm)
        val_g = -self.g * self.m * self.long * np.sin(tho)

        a = hp * (val_b * u - val_c * wo - val_g)

        thf = tho + wo * self.dt + (1 / 2) * a * self.dt ** 2

        wf = a * self.dt + wo

        return thf, wf

    def get_next_state(self, env, st):
        th, w, u = env
        for i in range(int(st/self.dt)):
            th, w = self.new_state_calculation(tho=th, wo=w, u=u)

        return [th, w]
