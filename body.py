# Hexapod Model
# Representation of the hexapod's shape and state.
# ALL UNITS IN MM
#
# Gaurav Manek

import numpy as np;
from pyrr import Quaternion, Vector3;
from movement import HexapodMotionPlanner;

class Hexapod(object):
    def __init__(self, config):
        self.body = HexapodBody(config);
        self.legs = [HexapodLeg(config, self.body, legid) for legid in config.getLegs()];
        self.mp = HexapodMotionPlanner(config, self.body, self.legs);
        
    def tick(self):
        self.mp.tick();

class HexapodBody(object):
    def __init__(self, config):
        self.init_rot = Quaternion.from_eulers([0., 0., 0.]); # Roll, Pitch, Yaw
        self.init_trans = Vector3([0., 0., 40.]);
        
        self.rot = self.init_rot;
        self.trans = self.init_trans;
    
    def getLegSegments(self, config, lobj):
        legargs = config.getLegSegmentConfig(lobj.getId())
            
        segs = [];
        prevSeg = lobj;

        for args in legargs:
            prevSeg = HexapodLegSegment(prevSeg, *args);
            segs.append(prevSeg);
        return segs;
        
    # Returns -1 if left side, 1 otherwise.
    def getLegDirection(self, leg):
        if "left" in leg:
            return -1;
        else:
            return 1;
        
    def getSkeletonPosition(self):
        return [self.getTranslation(), self.getRotation()];
    def getInitialRotation(self):
        return self.init_rot;
    def getRotation(self):
        return self.rot;
    def getInitialTranslation(self):
        return self.init_trans;
    def getTranslation(self):
        return self.trans;
        
    def setRotation(self, yaw, pitch, roll):
        self.rot = self.init_rot * Quaternion.from_eulers([yaw, pitch, roll]);
    def setTranslation(self, mtrans):
        self.trans = self.init_trans + mtrans;


class HexapodLegSegment(object):
    def __init__(self, prev_segment, *args):
        if len(args) == 1:
            if type(args[0]) is not HexapodLegSegment:
                raise TypeError("Copy constructor expects a single HexapodLegSegment argument.");
            o = args[0];
            
            self.prev = prev_segment;
            self.angle_map = o.angle_map;            
            self.rot_ax = o.rot_ax;
            self.disp = o.disp;
            self.pre_rot = o.pre_rot;
            self.st_ang = o.st_ang;
            self.angle_raw = o.angle_raw;
            self.angle = o.angle;
        elif len(args) == 5:
            angle_map, rotation_axis, disp, pre_rot, start_angle = args;
            self.angle_map = angle_map;        
            self.prev = prev_segment;
            self.rot_ax = rotation_axis;
            self.disp = disp;
            self.pre_rot = pre_rot;
            self.st_ang = start_angle;
            self.angle_raw = None;
            self.angle = None;
            self.setRotation(start_angle);
        else:
            raise TypeError("Incorrect arguments");
    
    def getRotation(self):
        return self.angle_raw;
        
    def setRotation(self, new_angle):
        #self.angle_raw = self.angle_map(new_angle);
        self.angle_raw = new_angle;
        self.angle = Quaternion.from_eulers([x * self.angle_raw for x in self.rot_ax]);
    
    def computeForwardKinematics(self):
        s_pos, s_rot, joint_axis = self.prev.computeForwardKinematics();
        p_pos, p_rot = s_pos[-1], s_rot[-1];
        
        c_rot = p_rot * self.pre_rot * self.angle;
        c_pos = p_pos + c_rot * self.disp;
        ja = p_rot * self.pre_rot * Vector3(self.rot_ax);
        
        return (s_pos + [c_pos]), (s_rot + [c_rot]), (joint_axis + [ja]);
    
    def clone(self, prevSeg):
        return HexapodLegSegment(prevSeg, self);
        

class HexapodLeg(object):
    def __init__(self, *args):
        if len(args) == 1:
            if type(args[0]) is not HexapodLeg:
                raise TypeError("Copy constructor expects a single HexapodLeg argument.");
                
            o = args[0];
            self.id = o.id;
            self.body = o.body;
            self.displacement = o.displacement;
            self.segments = [];
            prevSeg = self;
            for seg in o.segments:
                nextSeg = seg.clone(prevSeg);
                self.segments.append(nextSeg);
                prevSeg = nextSeg;
                
        elif len(args) == 3:
            config, body, legid = args;
            self.id = legid;
            self.body = body;
            self.displacement = config.getLegDisplacement(self);
            self.segments = body.getLegSegments(config, self);
        else:
            raise TypeError("Unexpected");
        
    def getId(self):
        return self.id;
    
    def getSegments(self):
        return self.segments;
    
    def getEndEffector(self):
        return self.segments[-1];
    
    def getEndEffectorPosition(self):
        s_pos, s_rot, joint_axis = self.getEndEffector().computeForwardKinematics();
        return s_pos[-1];
        
    def update(self, angles):
        for s,a in zip(self.segments, angles):
            s.setRotation(a);
    
    # Get a position fixed to the world, so that it can be used to plan steps.
    def getWorldPositionAnchor(self):
        itrans = self.body.getInitialTranslation();
        irot = self.body.getInitialRotation();
        return irot * self.displacement + itrans;
        
    def getRootPosition(self):
        itrans = self.body.getTranslation();
        irot = self.body.getRotation();
        return irot * self.displacement + itrans, irot;

    def computeForwardKinematics(self):
        rtp, rtr = self.getRootPosition();
        return ([rtp],[rtr], []);
    
    def clone(self):
        return HexapodLeg(self);
    
    def computeInverseKinematicsPass(self):
        s_pos, s_rot, joint_axis = self.getEndEffector().computeForwardKinematics();
        # Get the end effector position:
        ee = s_pos[-1];
        # For each joint, calculate the Jacobian:
        jacob = [(c_rot ^ (ee - p_pos)).xyz for p_pos, c_rot in zip(s_pos[:-1], joint_axis)];
        jacob = np.matrix(jacob);
        jacob_t = np.linalg.pinv(jacob);
        return s_pos, jacob_t;
        