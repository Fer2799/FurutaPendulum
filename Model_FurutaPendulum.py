import numpy as np


class FurutaPendulum:

    def __init__(self,
                 m1=100e-3,  # Mass of the arm
                 m2=200e-3,  # Mass of the pendulum
                 l1=100e-3,  # Length of the arm
                 l2=200e-3,  # Length of the pendulum
                 kt=27.8e-3,  # Torque constant
                 km=277.6e-4,  # Back-electromotive force constant
                 rm=9.73,  # Terminal resistance
                 c1=0.001,  # Damping coefficient of the arm
                 c2=0.001  # Damping coefficient of the pendulum
                 ):
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.i1 = (1 / 3) * m1 * l1 ** 2  # Moment of inertia of the arm
        self.i2 = (1 / 3) * m2 * l2 ** 2  # Moment of inertia of the pendulum
        self.g = 9.81  # Magnitude of the gravity
        self.kt = kt
        self.km = km
        self.rm = rm
        self.c1 = c1
        self.c2 = c2
        self.dt = 1e-3  # Constant time difference between states

    def delta(self, th2):
        d1 = self.i2 * self.i1
        d2 = self.m2 ** 2 * self.l2 ** 4 * np.sin(th2) ** 2
        d3 = self.m2 ** 2 * self.l2 ** 2 * self.l1 ** 2
        d4 = self.i1 * self.m2 * self.l2 ** 2
        d5 = self.i2 * self.m2 * self.l1 ** 2
        d6 = self.i2 * self.l2 ** 2 * self.m2 * np.sin(th2) ** 2
        d7 = self.l2 ** 2 * self.m2 ** 2 * self.l1 ** 2 * np.cos(th2) ** 2
        d = d1 + d2 + d3 + d4 + d5 + d6 - d7

        return d

    def hp(self, th2):
        a11 = self.m2 * self.l2 ** 2 + self.i2
        a12 = -self.l1 * self.l2 * self.m2 * np.cos(th2)
        a21 = -self.l1 * self.l2 * self.m2 * np.cos(th2)
        a22 = self.m2 * self.l1 ** 2 + self.m2 * self.l2 ** 2 * np.sin(th2)**2 + self.i1
        a = [[a11, a12],
             [a21, a22]]

        return a

    def matrix_calculation(self, th2, w1, w2):
        delta = 1 / self.delta(th2=th2)
        h_inv = np.multiply(self.hp(th2=th2), delta)  # Inverse matrix of H(q)

        mat_a = np.matmul(h_inv, [[self.kt/self.rm], [0]])

        c11 = self.km * self.kt / self.rm + (1/2) * self.l2 ** 2 * self.m2 * np.sin(2*th2) * w2 + self.c1
        c12 = (1/2) * self.l2 ** 2 * self.m2 * np.sin(2*th2) * w1 - self.l1 * self.l2 * self.m2 * np.sin(th2) * w2
        c21 = -(1/2) * self.l2 ** 2 * self.m2 * np.sin(2*th2) * w1
        c22 = self.c2
        c = [[c11, c12],
             [c21, c22]]
        mat_b = np.matmul(h_inv, c)

        mat_c = np.matmul(h_inv, [[0], [-self.g * self.l2 * self.m2 * np.sin(th2)]])

        return mat_a, mat_b, mat_c

    def new_state_calculation(self, th1o, th2o, w1o, w2o, u):

        a, b, c = self.matrix_calculation(th2=th2o, w1=w1o, w2=w2o)

        a1, a2 = np.multiply(a, u) - np.matmul(b, [[w1o], [w2o]]) - c

        th1f = th1o + w1o * self.dt + (1 / 2) * a1[0] * self.dt ** 2
        th2f = th2o + w2o * self.dt + (1 / 2) * a2[0] * self.dt ** 2

        w1f = a1[0] * self.dt + w1o
        w2f = a2[0] * self.dt + w2o

        return th1f, th2f, w1f, w2f

    def get_next(self, env, st):
        th1, th2, w1, w2, u = env
        for i in range(int(st/self.dt)):
            th1, th2, w1, w2 = self.new_state_calculation(th1o=th1, th2o=th2, w1o=w1, w2o=w2, u=u)

        return [th1, th2, w1, w2]
