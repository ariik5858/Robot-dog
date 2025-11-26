import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from ik_test import IKCalc

# ---------------- FK & plotting ---------------- #

def dh(a, d, alpha, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct, -st, 0,  a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa,  ca,  ca*d],
        [0, 0, 0, 1],
    ], dtype=float)

def fk_joint_positions(theta1, theta2, theta3):
    """Return positions of joints J0..J4 (5x3 array)."""
    a1, a2, a3, d1 = 25.5, 100.0, 100.21, 50.7

    # DH table you used earlier
    A1 = dh(0.0,   0.0, 0.0,        theta1)
    A2 = dh(a1,   d1,  -math.pi/2,  theta2)
    A3 = dh(a2,   0.0,  0.0,        theta3)
    A4 = dh(a3,   0.0,  0.0,        0.0)

    T01 = A1
    T02 = T01 @ A2
    T03 = T02 @ A3
    T04 = T03 @ A4

    p0 = np.array([0, 0, 0, 1.0])
    p1 = T01 @ p0
    p2 = T02 @ p0
    p3 = T03 @ p0
    p4 = T04 @ p0

    return np.vstack([p0[:3], p1[:3], p2[:3], p3[:3], p4[:3]])

def fk_joint_frames(theta1, theta2, theta3):
    """Return positions of joints J0..J4 (5x3 array)."""
    a1, a2, a3, d1 = 25.5, 100.0, 100.21, 50.7

    A1 = dh(0.0,   0.0,  0.0,         theta1)
    A2 = dh(a1,    d1,  -math.pi/2,   theta2)
    A3 = dh(a2,    0.0,  0.0,         theta3)
    A4 = dh(a3,    0.0,  0.0,         0.0)

    T01 = A1
    T02 = T01 @ A2
    T03 = T02 @ A3
    T04 = T03 @ A4

    Ts = [np.eye(4), T01, T02, T03, T04]
    pts = np.vstack([T[:3, 3] for T in Ts])

    return pts, Ts

# ---------------- Interactive GUI ---------------- #

def ik_gui():
    # initial goal
    x0, y0, z0 = -174.71,50.7,0
    mode1 = True   # theta1 branch
    mode2 = True   # elbow branch

    ik = IKCalc(x0, y0, z0)
    t1, t2, t3 = ik.solve(mode1, mode2)
    pts = fk_joint_positions(t1, t2, t3)

    plt.close("all")
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    # ADD HERE
    L = 50
    ax.quiver(0,0,0, L,0,0, color="r")
    ax.quiver(0,0,0, 0,L,0, color="g")
    ax.quiver(0,0,0, 0,0,L, color="b")
    ax.text(L,0,0,"X₀", color="r")
    ax.text(0,L,0,"Y₀", color="g")
    ax.text(0,0,L,"Z₀", color="b")

    plt.subplots_adjust(left=0.25, bottom=0.25)

    # initial plot
    line, = ax.plot(pts[:,0], pts[:,1], pts[:,2], marker="o")
    joint_labels = [ax.text(x, y, z, f"J{i}", fontsize=8) for i, (x, y, z) in enumerate(pts)]
    title = ax.set_title("IK visualizer")

    def set_axes_equal(pts_local):
        xyz_min, xyz_max = pts_local.min(axis=0), pts_local.max(axis=0)
        span = max((xyz_max - xyz_min).max(), 1.0)
        mid = (xyz_max + xyz_min) / 2
        ax.set_xlim(mid[0]-span/2, mid[0]+span/2)
        ax.set_ylim(mid[1]-span/2, mid[1]+span/2)
        ax.set_zlim(mid[2]-span/2, mid[2]+span/2)

    set_axes_equal(pts)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    # sliders for x, y, z
    axcolor = "lightgoldenrodyellow"
    ax_x = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
    ax_y = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor=axcolor)
    ax_z = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)

    s_x = Slider(ax_x, "x", -200.0, 200.0, valinit=x0)
    s_y = Slider(ax_y, "y", -200.0, 200.0, valinit=y0)
    s_z = Slider(ax_z, "z", 0.0, 200.0, valinit=z0)

    # radio buttons for modes
    ax_mode1 = plt.axes([0.02, 0.55, 0.15, 0.15], facecolor=axcolor)
    ax_mode2 = plt.axes([0.02, 0.35, 0.15, 0.15], facecolor=axcolor)
    rb_mode1 = RadioButtons(ax_mode1, ("+ branch", "- branch"), active=0)
    rb_mode2 = RadioButtons(ax_mode2, ("elbow up", "elbow down"), active=0)

    def recompute(_=None):
        nonlocal mode1, mode2
        x = s_x.val
        y = s_y.val
        z = s_z.val
        mode1 = (rb_mode1.value_selected == "+ branch")
        mode2 = (rb_mode2.value_selected == "elbow up")

        ik = IKCalc(x, y, z)
        try:
            t1, t2, t3 = ik.solve(mode1, mode2)
        except ValueError:
            # unreachable or acos domain error
            title.set_text("Unreachable pose")
            fig.canvas.draw_idle()
            return

        pts = fk_joint_positions(t1, t2, t3)
        line.set_data(pts[:,0], pts[:,1])
        line.set_3d_properties(pts[:,2])

        # update joint labels
        for lab, (xx, yy, zz) in zip(joint_labels, pts):
            lab.set_position((xx, yy))
            lab.set_3d_properties(zz, zdir="z")

        set_axes_equal(pts)
        title.set_text(
            f"θ1={math.degrees(t1):.1f}°, "
            f"θ2={math.degrees(t2):.1f}°, "
            f"θ3={math.degrees(t3):.1f}°"
        )
        fig.canvas.draw_idle()

    # connect callbacks
    s_x.on_changed(recompute)
    s_y.on_changed(recompute)
    s_z.on_changed(recompute)
    rb_mode1.on_clicked(recompute)
    rb_mode2.on_clicked(recompute)

    recompute()
    plt.show()

def main():
    # initial goal
    x0, y0, z0 = -174.71, 50.7, 0
    mode1 = True   # theta1 branch
    mode2 = True   # elbow branch

    ik = IKCalc(x0, y0, z0)
    t1, t2, t3 = ik.solve(mode1, mode2)
    pts, frames = fk_joint_frames(t1, t2, t3)

    plt.close("all")
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")

    # base frame (frame 0) axes
    L0 = 50
    ax.quiver(0, 0, 0, L0, 0, 0, color="r")
    ax.quiver(0, 0, 0, 0, L0, 0, color="g")
    ax.quiver(0, 0, 0, 0, 0, L0, color="b")
    ax.text(L0, 0, 0, "X₀", color="r")
    ax.text(0, L0, 0, "Y₀", color="g")
    ax.text(0, 0, L0, "Z₀", color="b")

    plt.subplots_adjust(left=0.25, bottom=0.25)

    # initial plot
    line, = ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], marker="o")
    joint_labels = [ax.text(x, y, z, f"J{i}", fontsize=8) for i, (x, y, z) in enumerate(pts)]
    title = ax.set_title("IK visualizer")

    # will hold the axis lines for each joint so we can update them
    joint_axes_artists = []

    def set_axes_equal(pts_local):
        xyz_min, xyz_max = pts_local.min(axis=0), pts_local.max(axis=0)
        span = max((xyz_max - xyz_min).max(), 1.0)
        mid = (xyz_max + xyz_min) / 2
        ax.set_xlim(mid[0] - span/2, mid[0] + span/2)
        ax.set_ylim(mid[1] - span/2, mid[1] + span/2)
        ax.set_zlim(mid[2] - span/2, mid[2] + span/2)

    def draw_joint_frames(frames_local):
        """Draw a small XYZ axis at each joint frame using its rotation matrix."""
        nonlocal joint_axes_artists

        # remove old artists
        for art in joint_axes_artists:
            try:
                art.remove()
            except ValueError:
                # already removed by matplotlib, ignore
                pass
        joint_axes_artists = []

        L_joint = 30.0  # length of each local axis

        for Ti in frames_local:
            origin = Ti[:3, 3]
            R = Ti[:3, :3]  # columns are x,y,z axes of the frame in world coords

            for col, color in zip(range(3), ("r", "g", "b")):
                v = R[:, col]
                end = origin + L_joint * v
                # plot returns a list of Line3D objects
                (ln,) = ax.plot(
                    [origin[0], end[0]],
                    [origin[1], end[1]],
                    [origin[2], end[2]],
                    color=color,
                    linewidth=1,
                )
                joint_axes_artists.append(ln)

    set_axes_equal(pts)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    # sliders for x, y, z
    axcolor = "lightgoldenrodyellow"
    ax_x = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
    ax_y = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor=axcolor)
    ax_z = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)

    s_x = Slider(ax_x, "x", -200.0, 200.0, valinit=x0)
    s_y = Slider(ax_y, "y", -200.0, 200.0, valinit=y0)
    s_z = Slider(ax_z, "z", 0.0, 200.0, valinit=z0)

    # radio buttons for modes
    ax_mode1 = plt.axes([0.02, 0.55, 0.15, 0.15], facecolor=axcolor)
    ax_mode2 = plt.axes([0.02, 0.35, 0.15, 0.15], facecolor=axcolor)
    rb_mode1 = RadioButtons(ax_mode1, ("+ branch", "- branch"), active=0)
    rb_mode2 = RadioButtons(ax_mode2, ("elbow up", "elbow down"), active=0)

    def recompute(_=None):
        nonlocal mode1, mode2, joint_axes_artists

        x = s_x.val
        y = s_y.val
        z = s_z.val
        mode1 = (rb_mode1.value_selected == "+ branch")
        mode2 = (rb_mode2.value_selected == "elbow up")

        ik = IKCalc(x, y, z)
        try:
            t1, t2, t3 = ik.solve(mode1, mode2)
        except ValueError:
            # unreachable or acos domain error
            title.set_text("Unreachable pose")
            fig.canvas.draw_idle()
            return

        pts_local, frames_local = fk_joint_frames(t1, t2, t3)

        # update robot line
        line.set_data(pts_local[:, 0], pts_local[:, 1])
        line.set_3d_properties(pts_local[:, 2])

        # update joint labels
        for lab, (xx, yy, zz) in zip(joint_labels, pts_local):
            lab.set_position((xx, yy))
            lab.set_3d_properties(zz, zdir="z")

        # update joint axes
        draw_joint_frames(frames_local)

        set_axes_equal(pts_local)
        title.set_text(
            f"θ1={math.degrees(t1):.1f}°, "
            f"θ2={math.degrees(t2):.1f}°, "
            f"θ3={math.degrees(t3):.1f}°"
        )
        fig.canvas.draw_idle()

    # connect callbacks
    s_x.on_changed(recompute)
    s_y.on_changed(recompute)
    s_z.on_changed(recompute)
    rb_mode1.on_clicked(recompute)
    rb_mode2.on_clicked(recompute)

    # initial draw of joint frames
    draw_joint_frames(frames)
    recompute()
    plt.show()

if __name__ == "__main__":
    ik_gui()
    # main()