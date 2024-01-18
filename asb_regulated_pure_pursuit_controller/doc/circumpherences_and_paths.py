#! /usr/bin/python3

from numpy import *
import matplotlib.pyplot as plt

set_printoptions(formatter={'float': lambda x: f"{x:+.3f}"})
plt.rcParams['figure.figsize'] = [10, 10]


def tf(x, t, a):
    r_mat = array([
        [cos(-a), -sin(-a)],
        [sin(-a), cos(-a)]])
    return (x-t) @ r_mat.T


def tf_inv(x, t, a):
    r_mat = array([
        [cos(a), -sin(a)],
        [sin(a), cos(a)]])
    return x @ r_mat.T + t


def plot_arc(c, r, a_0, a_1, color):
    res = pi/32
    arc_angles = linspace(a_0, a_1, 100)
    # arc_angles = linspace(a_0, a_1, int(ceil((fabs(a_1 - a_0))/res)))
    arc_xs = c[0] + r * cos(arc_angles)
    arc_ys = c[1] + r * sin(arc_angles)
    plt.plot(arc_xs, arc_ys, color=color, lw=1)


fig, ax = plt.subplots()  # note we must use plt.subplots, not plt.subplot
# ax.set_xlim((-10, 10))
# ax.set_ylim((-1, 10))

path = array([
    [0., 0.],
    [0., 1.],
    [0., 2.],
    [0., 3.],
    [0., 4.],
    [0., 5.],
    # [1., 6.],
])
plt.plot(path[:, 0], path[:, 1], '.-')

for p in path[:]:
    print(f"\n")
    print(f"p:   {p}")
    t_x, t_y = tt = array([3, 0])
    p_x_o, p_y_o = p
    aa = 0

    p_r = tf(p, tt, aa)
    print(f"p_r:\t{p_r}")
    p_x_r, p_y_r = p_r
    c_x_r = (p_x_r ** 2 + p_y_r ** 2) / (2 * p_x_r) if fabs(p_x_r) > 0.001 else inf
    print(f"c_x_r:\t{c_x_r}")

    p_o = tf_inv(p_r, tt, aa)
    print(f"p_o:\t{p_o}")
    c_x_o, c_y_o = c_o = tf_inv(array([c_x_r, 0]), tt, aa)
    print(f"c_o:\t{c_o}")

    if c_x_r < inf:
        t_x_c, t_y_c = tt-c_o
        p_x_c, p_y_c = p-c_o
        print(f"tt-c_o: {t_x_c, t_y_c}")
        print(f"p-c_o: {p_x_c, p_y_c}")
        a_t = arctan2(t_y_c, t_x_c)
        if a_t < 0:
            a_t += 2*pi
        a_p = arctan2(p_y_c, p_x_c)
        if a_p < 0:
            a_p += 2*pi
        print(f"a_t: {a_t/pi} pi")
        print(f"a_p: {a_p/pi} pi")
        # plt.scatter(c_x_o, c_y_o)
        plot_arc(c=[c_x_o, c_y_o], r=fabs(c_x_r), a_0=a_t, a_1=a_p, color='blue')
    else:
        plt.plot([t_x, p_x_o], [t_y, p_y_o], color='blue', lw=1)

for p in path[:]:
    print(f"\n")
    print(f"p:   {p}")
    t_x, t_y = tt = array([1.5, 1.5])
    p_x_o, p_y_o = p
    aa = 4*pi/8

    p_r = tf(p, tt, aa)
    print(f"p_r:\t{p_r}")
    p_x_r, p_y_r = p_r
    c_x_r = (p_x_r ** 2 + p_y_r ** 2) / (2 * p_x_r) if fabs(p_x_r) > 0.001 else inf
    print(f"c_x_r:\t{c_x_r}")

    p_o = tf_inv(p_r, tt, aa)
    print(f"p_o:\t{p_o}")
    c_x_o, c_y_o = c_o = tf_inv(array([c_x_r, 0]), tt, aa)
    print(f"c_o:\t{c_o}")

    if c_x_r < inf:
        t_x_c, t_y_c = tt-c_o
        p_x_c, p_y_c = p-c_o
        print(f"tt-c_o: {t_x_c, t_y_c}")
        print(f"p-c_o: {p_x_c, p_y_c}")
        a_t = arctan2(t_y_c, t_x_c)
        if a_t < 0:
            a_t += 2*pi
        a_p = arctan2(p_y_c, p_x_c)
        if a_p < 0:
            a_p += 2*pi
        print(f"a_t: {a_t/pi} pi")
        print(f"a_p: {a_p/pi} pi")
        # plt.scatter(c_x_o, c_y_o)
        plot_arc(c=[c_x_o, c_y_o], r=fabs(c_x_r), a_0=a_t, a_1=a_p, color='blue')
    else:
        plt.plot([t_x, p_x_o], [t_y, p_y_o], color='blue', lw=1)

for p in path[:]:
    print(f"\n")
    print(f"p:   {p}")
    t_x, t_y = tt = array([0.5, 0])
    p_x_o, p_y_o = p
    aa = 0

    p_r = tf(p, tt, aa)
    print(f"p_r:\t{p_r}")
    p_x_r, p_y_r = p_r
    c_x_r = (p_x_r ** 2 + p_y_r ** 2) / (2 * p_x_r) if fabs(p_x_r) > 0.001 else inf
    print(f"c_x_r:\t{c_x_r}")

    p_o = tf_inv(p_r, tt, aa)
    print(f"p_o:\t{p_o}")
    c_x_o, c_y_o = c_o = tf_inv(array([c_x_r, 0]), tt, aa)
    print(f"c_o:\t{c_o}")

    if c_x_r < inf:
        t_x_c, t_y_c = tt-c_o
        p_x_c, p_y_c = p-c_o
        print(f"tt-c_o: {t_x_c, t_y_c}")
        print(f"p-c_o: {p_x_c, p_y_c}")
        a_t = arctan2(t_y_c, t_x_c)
        if a_t < 0:
            a_t += 2*pi
        a_p = arctan2(p_y_c, p_x_c)
        if a_p < 0:
            a_p += 2*pi
        print(f"a_t: {a_t/pi} pi")
        print(f"a_p: {a_p/pi} pi")
        # plt.scatter(c_x_o, c_y_o)
        plot_arc(c=[c_x_o, c_y_o], r=fabs(c_x_r), a_0=a_t, a_1=a_p, color='green')
    else:
        plt.plot([t_x, p_x_o], [t_y, p_y_o], color='green', lw=1)

ax.set_aspect('equal', 'box')
fig.tight_layout()
plt.show()
