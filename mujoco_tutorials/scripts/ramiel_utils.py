import sympy
import numpy as np
import cvxopt
from cvxopt import matrix
from scipy.spatial.transform import Rotation

"""Define functions to calculate tendon length and tendon Jacobian.
"""

# define variables to be evaluated symbolically by sympy
roll = sympy.Symbol("roll")
pitch = sympy.Symbol("pitch")
slide = sympy.Symbol("slide")

r_top = 0.12  # distance from center axis to tendon
r_bottom = 0.294
height_wire_exit = 0.048/2
r_leg_attachment = 0.017;
attachment_bottom = 1.026  # position of bottom attachment point (top attachment point position is 0)
slack_top = 0.1  # length at top where sliding is disabled
slack_bottom = 0.1

min_tension = 8 # Neutons. Minimum tension when calculating tension from torque. Use this to avoid slack wires.

def check_range(roll_, pitch_, slide_):
    """
    check that the pose is within range, as sanity check
    """
    max_rotate = 0.8
    ok = abs(roll_) < max_rotate
    ok &= abs(pitch_) < max_rotate
    ok &= slack_top <= slide_ <= attachment_bottom - slack_bottom
    return ok

def range_reward(roll_, pitch_, slide_):
    max_rotate = 0.8
    min_slide = slack_top
    max_slide = attachment_bottom - slack_bottom
    rp_thre = 0.3
    slide_high_thre = 0.05
    slide_low_thre = 0.1
    reward = 0.0
    if roll_ > max_rotate-rp_thre:
        reward += -10.0*(roll_-(max_rotate-rp_thre))**2
    if roll_ < -max_rotate+rp_thre:
        reward += -10.0*(roll_-(-max_rotate+rp_thre))**2
    if pitch_ > max_rotate-rp_thre:
        reward += -10.0*(pitch_-(max_rotate-rp_thre))**2
    if pitch_ < -max_rotate+rp_thre:
        reward += -10.0*(pitch_-(-max_rotate+rp_thre))**2
    if slide_ > max_slide-slide_high_thre:
        reward += -100.0*(slide_-(max_slide-slide_high_thre))**2
    if slide_ < min_slide+slide_low_thre:
        reward += -100.0*(slide_-(min_slide+slide_low_thre))**2
    return reward

def horizontal_eval(quat):
    r = Rotation.from_quat(quat).as_matrix()
    z = np.asarray([0, 0, 1])
    z_converted = r.dot(z)
    return z.dot(z_converted)

R_roll = sympy.Matrix([
    [1, 0, 0],
    [0, sympy.cos(roll), -sympy.sin(roll)],
    [0, sympy.sin(roll), sympy.cos(roll)]])
R_pitch = sympy.Matrix([
    [sympy.cos(pitch), 0, sympy.sin(pitch)],
    [0, 1, 0],
    [-sympy.sin(pitch), 0, sympy.cos(pitch)]])


xyz1_vertical_top = sympy.Matrix([r_leg_attachment*np.cos(np.pi*0/3), r_leg_attachment*np.sin(np.pi*0/3), slide])
xyz1_top = R_roll*R_pitch*xyz1_vertical_top
xyz2_vertical_top = sympy.Matrix([r_leg_attachment*np.cos(np.pi*2/3), r_leg_attachment*np.sin(np.pi*2/3), slide])
xyz2_top = R_roll*R_pitch*xyz2_vertical_top
xyz3_vertical_top = sympy.Matrix([r_leg_attachment*np.cos(np.pi*4/3), r_leg_attachment*np.sin(np.pi*4/3), slide])
xyz3_top = R_roll*R_pitch*xyz3_vertical_top
# print(f"top tendon attachment: ({xyz1_top}, {xyz2_top}, {xyz3_top})")

bottom_slide = slide - attachment_bottom
xyz1_vertical_bottom = sympy.Matrix([r_leg_attachment*np.cos(np.pi*0/3), r_leg_attachment*np.sin(np.pi*0/3), bottom_slide])
xyz1_bottom = R_roll*R_pitch*xyz1_vertical_bottom
xyz2_vertical_bottom = sympy.Matrix([r_leg_attachment*np.cos(np.pi*2/3), r_leg_attachment*np.sin(np.pi*2/3), bottom_slide])
xyz2_bottom = R_roll*R_pitch*xyz2_vertical_bottom
xyz3_vertical_bottom = sympy.Matrix([r_leg_attachment*np.cos(np.pi*4/3), r_leg_attachment*np.sin(np.pi*4/3), bottom_slide])
xyz3_bottom = R_roll*R_pitch*xyz3_vertical_bottom
# print(f"bottom tendon attachment: ({xyz1_bottom}, {xyz2_bottom}, {xyz3_bottom})")

def dist(x, y, z, r, theta, z2):
    """
    return distance between tendon attachment point at base & pole
    x, y, z: coordinates of tendon attachment position at pole
    r: distance bet. center axis and tendon attachment at base
    theta: angle of tendon attachment at base
    """
    return sympy.sqrt((x - r*np.cos(theta))**2 + (y - r*np.sin(theta))**2 + (z - z2)**2)

# calculate length of each tendon
l1_top = dist(xyz1_top[0], xyz1_top[1], xyz1_top[2], r_top, np.pi*0/3, height_wire_exit)
l2_top = dist(xyz2_top[0], xyz2_top[1], xyz2_top[2], r_top, np.pi*2/3, height_wire_exit)
l3_top = dist(xyz3_top[0], xyz3_top[1], xyz3_top[2], r_top, np.pi*4/3, height_wire_exit)

l1_bottom = dist(xyz1_bottom[0], xyz1_bottom[1], xyz1_bottom[2], r_bottom, np.pi*0/3, -height_wire_exit)
l2_bottom = dist(xyz2_bottom[0], xyz2_bottom[1], xyz2_bottom[2], r_bottom, np.pi*2/3, -height_wire_exit)
l3_bottom = dist(xyz3_bottom[0], xyz3_bottom[1], xyz3_bottom[2], r_bottom, np.pi*4/3, -height_wire_exit)

l = sympy.Matrix([l1_top, l2_top, l3_top, l1_bottom, l2_bottom, l3_bottom])
l_func = sympy.lambdify([roll, pitch, slide], l)  # calling l.subs is very slow so make it into a function
# define the Jacobian
J = l.jacobian(sympy.Matrix([roll, pitch, slide]))
J_func = sympy.lambdify([roll, pitch, slide], J)

def pose2tendon(roll_, pitch_, slide_):
    return np.array(l_func(roll_, pitch_, slide_)).astype(np.float64)


def pose2jacobian(roll_, pitch_, slide_):
    return np.array(J_func(roll_, pitch_, slide_)).astype(np.float64)


# define matrices that are constant in the QP formulation
P = matrix(np.identity(6))
q = matrix(np.zeros(6))
G = matrix(np.identity(6))
h = -min_tension * matrix(np.ones(6))
cvxopt.solvers.options['show_progress'] = False  # suppress output


def compute_tension(jacobian, tau):
    """
    jacobian: jacobian of six muscles against 3 joints
    tau: 3D vector of generalized forces
    solves QP to compute muscle force from generalized forces
    QP is formulated to
    - minimize squared sum of muscle tension
    - each muscle tension is negative
    - achieve generalized force
    """
    # define matrices for the equality constraint
    A = matrix(np.transpose(jacobian))
    tau_double = [float(t) for t in tau]
    b = matrix(tau_double)
    # solve QP
    sol = cvxopt.solvers.qp(P, q, G, h, A, b)
    return sol["x"]
