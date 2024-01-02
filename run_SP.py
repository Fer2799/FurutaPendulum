from Model_SinglePendulum import SinglePendulum
import numpy as np
import matplotlib.pyplot as plt

sp = SinglePendulum()

th, w = np.pi/2, 0

# print(t1, t2, w1, w2)

data = [[0, th, w]]
t_step = 0.1

for i in range(1, 300 + 1):
    if i < 10 + 1:
        th, w = sp.get_next_state([th, w, 5], t_step)
    else:
        th, w = sp.get_next_state([th, w, 0], t_step)

    t = t_step * i
    data.append([t, th, w])

print('Done.')
data = np.array(data)

fig, ax = plt.subplots()
ax2 = ax.twinx()

ax.plot(data[:, 0], data[:, 1], label=r'$\theta$')
ax2.plot(data[:, 0], data[:, 2], label=r'$\omega$', color='orange')

plt.title('Angular position and velocity of the pendulum \n'
          'with a sampling time of ' + str(t_step) + ' s')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Position [rad]')
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Velocity [rad/s]')
ax.figure.legend()
plt.gcf().set_dpi(300)
plt.show()
