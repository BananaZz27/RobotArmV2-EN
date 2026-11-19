# 3D view with PyVista embedded in Qt (adapted from ws-control/gui/ui/viewer_3d.py)
#   unit meshes (cylinder height 1, sphere radius 1) are built once; moving the arm only changes user_matrix → fast
#   Target arm (solid orange) = pose to command · Actual arm (translucent blue) = real pose (simulation: the moving pose)
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QVBoxLayout, QWidget

import config
from kinematics import forward_kinematics

LINK_RADIUS, JOINT_RADIUS, AXIS_LENGTH = 9.0, 13.0, 40.0
REACH = config.D1 + config.A2 + config.A3 + config.D5
ARROW_X_TO_Z = np.array([[0.0, 0, -1], [0, 1, 0], [1, 0, 0]])   # rotate an arrow pointing +x to point +z


def _align_z(v):
    """rotation matrix that points the z axis along v"""
    z = v / np.linalg.norm(v)
    helper = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _translate(p):
    M = np.eye(4)
    M[:3, 3] = p
    return M


class ArmVisual:
    def __init__(self, plotter, color, opacity, show_axes):
        cyl = pv.Cylinder(center=(0, 0, 0.5), direction=(0, 0, 1), radius=1.0, height=1.0, resolution=24)
        sphere = pv.Sphere(radius=1.0, theta_resolution=20, phi_resolution=20)
        n = len(forward_kinematics([0] * 5).frames) - 1
        style = dict(color=color, opacity=opacity, smooth_shading=True, pickable=False)
        self.links = [plotter.add_mesh(cyl.copy(), **style) for _ in range(n)]
        self.joints = [plotter.add_mesh(sphere.copy(), **{**style, "color": "#444444"}) for _ in range(n)]
        self.axes = [plotter.add_mesh(pv.Arrow(direction=(1, 0, 0), scale=AXIS_LENGTH), color=c, pickable=False)
                     for c in ("red", "green", "blue")] if show_axes else []

    def set_visible(self, on):
        for a in self.links + self.joints + self.axes:
            a.SetVisibility(on)

    def update(self, q):
        frames = forward_kinematics(q).frames
        origins = [T[:3, 3] for T in frames]
        for i, actor in enumerate(self.links):            # link i: origin i → origin i+1
            v = origins[i + 1] - origins[i]
            length = np.linalg.norm(v)
            actor.SetVisibility(bool(length > 1e-6))            # coincident frames (wrist) are not drawn
            if length > 1e-6:
                M = np.eye(4)
                M[:3, :3] = _align_z(v) @ np.diag([LINK_RADIUS, LINK_RADIUS, length])
                M[:3, 3] = origins[i]
                actor.user_matrix = M
        for i, actor in enumerate(self.joints):
            M = _translate(origins[i])
            M[:3, :3] *= JOINT_RADIUS
            actor.user_matrix = M
        for k, actor in enumerate(self.axes):              # x/y/z axes of the tip
            R = np.eye(4)
            R[:3, :3] = _align_z(frames[-1][:3, k]) @ ARROW_X_TO_Z
            R[:3, 3] = frames[-1][:3, 3]
            actor.user_matrix = R


class Viewer3D(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.plotter = p = QtInteractor(self)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(p.interactor)
        p.set_background("#f4f5f7", top="#dfe3ea")
        p.add_mesh(pv.Plane(i_size=2.4 * REACH, j_size=2.4 * REACH, i_resolution=12, j_resolution=12),
                   color="#c9ced6", style="wireframe", line_width=1, pickable=False)
        p.add_mesh(pv.Cylinder(center=(0, 0, 8), direction=(0, 0, 1), radius=45, height=16),
                   color="#5b6270", smooth_shading=True, pickable=False)
        p.add_axes(line_width=2)
        self.actual = ArmVisual(p, "#3a86ff", 0.35, False)
        self.target = ArmVisual(p, "#ff8c42", 1.0, True)
        self.actual.set_visible(False)
        self._marker = p.add_mesh(pv.Sphere(radius=7), color="#2ec4b6", pickable=False)
        self._marker.SetVisibility(False)
        self.target.update([0] * 5)
        p.camera_position = [(REACH * 1.6, -REACH * 1.6, REACH * 1.2), (0, 0, REACH * 0.35), (0, 0, 1)]
        self._dirty = True
        self._timer = QTimer(self, interval=33)            # redraw at most ~30 FPS
        self._timer.timeout.connect(self._render)
        self._timer.start()

    def show_target(self, q):
        self.target.update(q)
        self._dirty = True

    def show_actual(self, q):
        self.actual.set_visible(bool(q is not None))
        if q is not None:
            self.actual.update(q)
        self._dirty = True

    def show_ik_marker(self, xyz):
        self._marker.SetVisibility(bool(xyz is not None))
        if xyz is not None:
            self._marker.user_matrix = _translate(xyz)
        self._dirty = True

    def _render(self):
        if self._dirty:
            self._dirty = False
            self.plotter.render()

    def close(self):
        self._timer.stop()
        self.plotter.close()
        return super().close()
