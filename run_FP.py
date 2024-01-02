from Model_FurutaPendulum import FurutaPendulum
import numpy as np
import matplotlib.pyplot as plt

fp = FurutaPendulum()

t1, t2, w1, w2 = 0, 0, 0, 0

# print(t1, t2, w1, w2)

data = [[0, t1, t2, w1, w2]]
t_step = 0.001

for i in range(1, 30000 + 1):  # Time of simulation
    if i < 2 + 1:
        t1, t2, w1, w2 = fp.get_next([t1, t2, w1, w2, -5], t_step)
    else:
        t1, t2, w1, w2 = fp.get_next([t1, t2, w1, w2, 0], t_step)

    t = t_step * i
    data.append([t, t1, t2, w1, w2])
    # print(t1, t2, w1, w2)

print('Done.')
data = np.array(data)

plt.plot(data[:, 0], data[:, 1], label=r'$\theta_1$')
plt.plot(data[:, 0], data[:, 2], label=r'$\theta_2$')

plt.title('Position of the arm and pendulum '
          'with a sampling time of ' + str(t_step))
plt.xlabel('Time [s]')
plt.ylabel('Position [rad]')
plt.legend()
plt.gcf().set_dpi(300)
plt.show()
