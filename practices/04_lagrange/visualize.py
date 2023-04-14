from matplotlib import pyplot as plt, animation
import numpy as np
from collections import deque


def visualize_double_pendulum(points,
                              stats=None,
                              dt=0.01,
                              trace_len=0.1,
                              save=False,
                              verbose=True,
                              axes=False,
                              show=True):
    # TODO: Get pendulums lengths
    points_1, points_2 = points
    # print(points)
    x1, y1 = points_1
    x2, y2 = points_2
    LEN = len(x1)

    L1 = (x1[0]**2 + y1[0]**2)**0.5
    L2 = ((x2[0] - x1[0])**2 + (y2[0] - y1[0])**2)**0.5
    L_MAX = 1.1*(L2 + L1)

    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(autoscale_on=False,
                         xlim=(-L_MAX, L_MAX), ylim=(-L_MAX, L_MAX))
    # ax.set_aspect('equal')
    ax.set_aspect('equal', adjustable='box')
    # plt.axis('off')
    if axes:
        # plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        # plt.grid(True)
        ax.grid(color='black', linestyle='--', linewidth=0.8, alpha=0.5)
    else:
        plt.axis('off')

    line, = ax.plot([], [], '-', c='black', lw=2)
    circle_1, = ax.plot([], [], 'o', c='blue', markersize=6, zorder=5)
    circle_2, = ax.plot([], [], 'o', c='red', markersize=6, zorder=5)

    trace, = ax.plot([], [], '-', c='r', lw=1, alpha=0.2)

    corner_text = ax.text(0.05, 0.8, '', transform=ax.transAxes, zorder=10)
    history_x, history_y = deque(maxlen=int(
        trace_len*LEN)), deque(maxlen=int(trace_len*LEN))

    def animate(i):
        stat_text = ''
        thisx = [0, x1[i], x2[i]]
        thisy = [0, y1[i], y2[i]]

        if i == 0:
            history_x.clear()
            history_y.clear()

        history_x.appendleft(thisx[2])
        history_y.appendleft(thisy[2])

        line.set_data(thisx, thisy)
        circle_1.set_data(x1[i], y1[i])
        circle_2.set_data(x2[i], y2[i])
        # line.set_data(thisx, thisy)
        trace.set_data(history_x, history_y)

        if stats is not None:
            for stat in stats:
                stat_text += rf'{stat[0]}:  {round(stat[1][i],3)}'
                stat_text += '\n'
            corner_text.set_text(stat_text)

        return circle_1, circle_2, line, trace, corner_text

    ani = animation.FuncAnimation(
        fig, animate, interval=dt*500, frames=LEN, blit=True)
    if save:
        if verbose:
            print('Animation being saved...')
        # print('Hit CTRL+W to exit')
        # ani.save('pendulum.gif', writer='pillow', fps = 1/dt)
        with open('double_pendulum.html','w') as f:
            f.write(ani.to_jshtml())

    if verbose:
        print('Animation begin...')
        print('Hit CTRL+W to exit')
    if show:
        plt.show()

    return ani
