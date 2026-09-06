# -*- coding: utf-8 -*-
"""Python 2.7/3 shared math and LOCAL motion guards; not a collision checker."""
from __future__ import division
import math
import os
import time


def clock():
    # Linux os.times elapsed real time has a fixed origin, unlike wall clock.
    return time.monotonic() if hasattr(time, 'monotonic') else os.times()[4]


def vector(value, n=6):
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise ValueError('Expected %d numbers' % n)
    if any(isinstance(x, bool) for x in value):
        raise ValueError('Boolean is not a coordinate')
    out = [float(x) for x in value]
    if any(math.isnan(x) or math.isinf(x) for x in out):
        raise ValueError('Coordinates must be finite')
    return out


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rotation(p):
    z, y, x = [math.radians(v) for v in p[3:]]
    cz, sz, cy, sy, cx, sx = (math.cos(z), math.sin(z), math.cos(y),
                             math.sin(y), math.cos(x), math.sin(x))
    return matmul(matmul([[cz,-sz,0],[sz,cz,0],[0,0,1]],
                         [[cy,0,sy],[0,1,0],[-sy,0,cy]]),
                         [[1,0,0],[0,cx,-sx],[0,sx,cx]])


def angle(a, b):
    ra, rb = rotation(a), rotation(b)
    trace = sum(ra[i][j]*rb[i][j] for i in range(3) for j in range(3))
    return math.degrees(math.acos(max(-1., min(1., (trace-1.)/2.))))


def distance(a, b):
    return norm([a[i]-b[i] for i in range(3)])


def offset_pose(pose, delta):
    """delta=[dx,dy,dz,drz,dry,drx], rotations about BASE axes at flange."""
    pose, delta = vector(pose), vector(delta)
    r = matmul(rotation([0,0,0]+delta[3:]), rotation(pose))
    y = math.asin(max(-1., min(1., -r[2][0])))
    if abs(math.cos(y)) < 0.05:
        raise ValueError('Euler representation near gimbal lock; reposition by teaching')
    a = [math.degrees(math.atan2(r[1][0],r[0][0])), math.degrees(y),
         math.degrees(math.atan2(r[2][1],r[2][2]))]
    # Choose equivalent Euler family closest to the current representation.
    families = [a, [a[0]+180., 180.-a[1], a[2]+180.]]
    candidates = [[v+360.*round((pose[i+3]-v)/360.) for i,v in enumerate(f)]
                  for f in families]
    a = min(candidates, key=lambda f: sum((f[i]-pose[i+3])**2 for i in range(3)))
    return [pose[i]+delta[i] for i in range(3)] + a


class Guard(object):
    """Application ceilings, NOT manufacturer joint limits or certified safety."""
    limits = dict(max_step_mm=2., max_step_deg=1., anchor_radius_mm=20.,
                  anchor_angle_deg=10., anchor_joint_deg=10.,
                  max_joint_step_deg=2., max_ik_sample_jump_deg=1.,
                  sample_mm=0.5, sample_deg=0.25)

    def __init__(self, joint_limits=None):
        self.joint_limits=None
        if joint_limits is not None:
            if not isinstance(joint_limits,dict) or joint_limits.get('verified') is not True:
                raise ValueError('Joint limits must be verified from this robot/controller')
            low,high=vector(joint_limits.get('min_deg')),vector(joint_limits.get('max_deg'))
            margin=float(joint_limits.get('margin_deg',0.))
            if not 0<=margin<90 or any(a+margin>=b-margin for a,b in zip(low,high)):
                raise ValueError('Invalid joint limit interval/margin')
            self.joint_limits=dict(min_deg=low,max_deg=high,margin_deg=margin,verified=True)

    def path(self, current, target, anchor):
        current, target, anchor = vector(current), vector(target), vector(anchor)
        for name, value, bound in (
                ('step translation',distance(current,target),self.limits['max_step_mm']),
                ('step rotation',angle(current,target),self.limits['max_step_deg']),
                ('anchor translation',distance(anchor,target),self.limits['anchor_radius_mm']),
                ('anchor rotation',angle(anchor,target),self.limits['anchor_angle_deg'])):
            if value > bound+1e-7:
                raise ValueError('%s %.4f exceeds application limit %.4f' % (name,value,bound))
        if max(abs(target[i]-current[i]) for i in range(3,6)) > 3.:
            raise ValueError('Euler discontinuity or large component change')
        count = max(1, int(math.ceil(distance(current,target)/self.limits['sample_mm'])),
                    int(math.ceil(max(abs(target[i]-current[i]) for i in range(3,6)) /
                                  self.limits['sample_deg'])))
        path = [[current[i]+(target[i]-current[i])*k/count for i in range(6)]
                for k in range(count+1)]
        for p in path:
            if abs(math.cos(math.radians(p[4]))) < 0.05:
                raise ValueError('Euler representation near gimbal lock')
            if distance(anchor,p)>self.limits['anchor_radius_mm']+1e-7 or \
                    angle(anchor,p)>self.limits['anchor_angle_deg']+1e-7:
                raise ValueError('Intermediate pose leaves local envelope')
        return path

    def check_joints(self, path_joints, current, anchor):
        current, anchor = vector(current), vector(anchor)
        prev = current
        for item in path_joints:
            q = vector(item)
            if self.joint_limits:
                limits=self.joint_limits
                for i,v in enumerate(q):
                    if not limits['min_deg'][i]+limits['margin_deg']<=v<=limits['max_deg'][i]-limits['margin_deg']:
                        raise ValueError('Joint %d exceeds configured limit/margin' % (i+1))
            # Deliberately no modulo 360: cable wrap/multiturn is significant.
            if max(abs(q[i]-anchor[i]) for i in range(6)) > self.limits['anchor_joint_deg']:
                raise ValueError('Joint leaves anchor corridor')
            if max(abs(q[i]-current[i]) for i in range(6)) > self.limits['max_joint_step_deg']:
                raise ValueError('Excessive joint change for this short Cartesian move')
            if max(abs(q[i]-prev[i]) for i in range(6)) > self.limits['max_ik_sample_jump_deg']:
                raise ValueError('IK discontinuity / large local joint response')
            prev = q


def quality_plan(anchor, translation_mm=1., rotation_deg=0.5):
    """Twelve axis probes, each followed by the anchor. No accumulated drift."""
    translation_mm, rotation_deg = float(translation_mm), float(rotation_deg)
    if not (0 < translation_mm <= 2. and 0 < rotation_deg <= 1.):
        raise ValueError('plan steps: 0 < mm <= 2, 0 < deg <= 1')
    result = []
    for axis, name in enumerate(('x','y','z','rz','ry','rx')):
        for sign in (1,-1):
            d = [0.]*6
            d[axis] = sign*(translation_mm if axis<3 else rotation_deg)
            result.append(dict(label='%s%+g' % (name,d[axis]),
                               target=offset_pose(anchor,d), capture=True))
            result.append(dict(label='anchor', target=list(anchor), capture=False))
    return result
