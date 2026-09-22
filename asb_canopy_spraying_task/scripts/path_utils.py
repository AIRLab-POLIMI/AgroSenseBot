#!/usr/bin/python3

import numpy as np
import numpy.typing as npt

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def quaternion_from_yaw(yaw: float):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def norm_angle(theta):
    return (theta + 2*np.pi)  %  (2*np.pi)

def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n == 0:
        raise ValueError("zero-length vector")
    return v / n

def perp(v):
    """2D perpendicular rotated +90 degrees: (x,y) -> (-y, x)"""
    return np.array([-v[1], v[0]])

def cross2d(a: np.array, b: np.array):
    return a[0] * b[1] - a[1] * b[0]

class AlignedSegments(ValueError):
    pass

class OppositeSegments(ValueError):
    pass

def circle_center_and_tangents(p1, p2, p3, R, tol=1e-9):
    """
    Given 2D points p1, p2, p3 and radius R,
    - s1 is the ray starting at p2 and passing through p1 (direction v1 = p1 - p2)
    - s2 is the ray starting at p2 and passing through p3 (direction v2 = p3 - p2)
    Finds center C of circle of radius R that is tangent to both rays on their acute side,
    and returns tangent points t1 (on s1) and t2 (on s2).
    Returns: C, t1, t2, theta_start, theta_end
    Raises AlignedSegments if the segments (p1 -> p2) and (p2 -> p3) are aligned and the arc is not needed.
    Raises OppositeSegments if the segments (p1 -> p2) and (p2 -> p3) are parallel and overlapping, and the arc cannot be computed.
    Raises ValueError if there is no solution for other reasons.
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    p3 = np.asarray(p3, dtype=float)

    v1 = p1 - p2
    v2 = p3 - p2

    # unit directions along the rays
    u1 = unit(v1)
    u2 = unit(v2)

    # interior (acute) bisector direction; if u1+u2 is near zero, the segments p1 -> p2 and p2 -> p3 are aligned
    bis = u1 + u2
    if np.linalg.norm(bis) < tol:
        # no arc needed to join the segments
        raise AlignedSegments("rays are opposite (angle ~180°); acute side ambiguous")

    if abs(cross2d(u1, u2)) < tol:
        # nearly-parallel rays, it is not possible to bevel
        raise OppositeSegments("rays are parallel")

    bis = unit(bis)

    # Normals for offset lines: take perp(u1) and perp(u2) and flip sign so they point into interior (toward bis)
    n1 = perp(u1)
    if np.dot(n1, bis) < 0:
        n1 = -n1
    n1 = unit(n1)

    n2 = perp(u2)
    if np.dot(n2, bis) < 0:
        n2 = -n2
    n2 = unit(n2)

    # parametric lines:
    # l1(s) = p2 + n1*R + s*u1
    # l2(t) = p2 + n2*R + t*u2
    # solve p2 + n1*R + s*u1 = p2 + n2*R + t*u2
    # => s*u1 - t*u2 = (n2 - n1) * R

    A = np.column_stack([u1, -u2])  # 2x2
    rhs = (n2 - n1) * R

    try:
        sol = np.linalg.solve(A, rhs)
    except np.linalg.LinAlgError as e:
        raise ValueError("linear system singular: rays may be parallel or nearly so") from e

    s = sol[0]

    center = p2 + n1 * R + s * u1

    # tangent points are projections of center onto the original rays (lines through p2, dir u1/u2)
    proj_len1 = np.dot(center - p2, u1)
    proj_len2 = np.dot(center - p2, u2)
    t1 = p2 + proj_len1 * u1
    t2 = p2 + proj_len2 * u2

    theta1 = np.arctan2((t1 - center)[1], (t1 - center)[0])
    theta2 = np.arctan2((t2 - center)[1], (t2 - center)[0])

    if norm_angle(theta2 - theta1) < norm_angle(theta1 - theta2):
        theta_diff = norm_angle(theta2 - theta1)
        theta_start, theta_end = norm_angle(theta1), norm_angle(theta1) + theta_diff
    else:
        theta_diff = norm_angle(theta1 - theta2)
        theta_start, theta_end = norm_angle(theta1), norm_angle(theta1) - theta_diff

    return center, t1, t2, theta_start, theta_end

def interpolate_arc(center, radius, start_angle, end_angle, path_pose_distance):
    arc_len = abs(end_angle - start_angle) * radius
    num_points = max(3, int(np.ceil(arc_len / path_pose_distance)) + 1)
    angles = np.linspace(start_angle, end_angle, num_points)
    return [
        np.array([
            center[0] + radius * np.cos(a),
            center[1] + radius * np.sin(a)
            ]) for a in angles]

def interpolate_line(p1, p2, path_pose_distance):
    seg_len = np.linalg.norm(p2 - p1)
    num_points = max(2, int(np.ceil(seg_len / path_pose_distance)) + 1)
    return list(np.linspace(p1, p2, num=num_points, endpoint=False))[1:]

def points_to_beveled_path(points: npt.NDArray[np.float64], min_turning_radius, path_pose_distance):

    arcs = [[None, points[0], []]]
    for i in range(1, len(points) - 1):
        p1, p2, p3 = points[i - 1], points[i], points[i + 1]
        try:
            c, t1, t2, theta_start, theta_end = circle_center_and_tangents(p1, p2, p3, min_turning_radius)
            arc_points = interpolate_arc(c, min_turning_radius, theta_start, theta_end, path_pose_distance)
            arc = [t1, t2, arc_points]
            arcs.append(arc)
        except AlignedSegments:
            arc = [t1, t2, []]
        except OppositeSegments:
            print("opposite segments, arc cannot be computed (NOT OK).")
    arcs.append([points[-1], None, []])

    path_points = [points[0]]
    for [_, arc1_end_point, _], [arc2_start_point, _, arc2_points] in zip(arcs[0:-1], arcs[1:]):
        line_path = interpolate_line(arc1_end_point, arc2_start_point, path_pose_distance)
        path_points += line_path
        path_points += arc2_points
    path_points += [points[-1]]

    return np.column_stack(path_points)
