# Hexapod Model
# Representation of the hexapod's shape and state.
# ALL UNITS IN MM
#
# Gaurav Manek

import math;
from pyrr import Quaternion, Vector3;

class HexapodBody(object):
    def __init__(self):
        self.init_rot = Quaternion.from_eulers([0., 0., 0.]); # Roll, Pitch, Yaw
        self.init_trans = Vector3([0., 0., 40.]);
        
        self.rot = self.init_rot;
        self.trans = self.init_trans;    
    
    def getLegDisplacement(self, lobj):
        leg = lobj.getId();
        
        width_end = 63./2;
        width_mid = 84./2;
        length = 113./2;
        
        x = 0;
        y = 0;
        
        if "front" in leg:
            y = length;
        elif "rear" in leg:
            y = -length;
        else:
            y = 0;
        
        if "left" in leg:
            x = -width_mid if y == 0 else -width_end;
        else:
            x = width_mid if y == 0 else width_end;
            
        return Vector3([x, y, 0.]);
        
    def getLegSegments(self, lobj):
        leg = lobj.getId();
        
        ymult = 1;
        change_angle = lambda x: x;
        if "left" in leg:
            ymult = -1;
            change_angle = lambda x: math.pi - x;
            
        segs = [];
        prevSeg = lobj;

        #[rotation_axis, disp, start_angle]
        legargs = [ [[1., 0., 0.], Vector3([13.3, 14.9, 0]), Quaternion.from_eulers([0., 0., math.pi/2]), change_angle(3*math.pi/2)],
                    [[0., 1., 0.], Vector3([0, 55.4, 0]), Quaternion.from_eulers([0, 0, 0]), change_angle(math.asin(30.1/55.4))],
                    [[0., 1., 0.], Vector3([0, 90.0, 0]), Quaternion.from_eulers([0, 0, 0]), change_angle(math.pi/2)]
                    ];
        
        for args in legargs:
            prevSeg = HexapodLegSegment(prevSeg, *args);
            segs.append(prevSeg);
        #segs.reverse();
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
    def __init__(self, prev_segment, rotation_axis, disp, pre_rot, start_angle):
        self.prev = prev_segment;
        self.rot_ax = rotation_axis;
        self.disp = disp;
        self.pre_rot = pre_rot;
        self.st_ang = start_angle;
        self.angle_raw = None;
        self.angle = None;
        self.setRotation(start_angle);
        
    def setRotation(self, new_angle):
        self.angle_raw = new_angle;
        self.angle = Quaternion.from_eulers([x * new_angle for x in self.rot_ax]);
    
    def getSegmentPosition(self):
        p_pos, p_rot = None, None;
        if type(self.prev) is HexapodLegSegment:
            p_pos, p_rot = self.prev.getSegmentPosition();
        elif type(self.prev) is HexapodLeg:
            p_pos, p_rot = self.prev.getRootPosition();
            
        c_rot = p_rot * self.pre_rot;
        c_pos = p_pos + c_rot * self.disp;
        return c_pos, c_rot;
        
    def getSkeletonPosition(self):
        s_pos, s_rot = self.prev.getSkeletonPosition();        
        p_pos, p_rot = s_pos[-1], s_rot[-1];
        
        c_rot = p_rot * self.pre_rot;
        c_pos = p_pos + c_rot * self.disp;
        c_rot = c_rot * self.angle;
        return (s_pos + [c_pos]), (s_rot + [c_rot]);
        

class HexapodLeg(object):
    def __init__(self, body, legid):
        self.id = legid;
        self.body = body;
        self.displacement = body.getLegDisplacement(self);
        self.segments = body.getLegSegments(self);
        
    def getId(self):
        return self.id;
    
    def getSegments(self):
        return self.segments;
    
    def getEndEffector(self):
        return self.segments[-1];
    
    # Get a position fixed to the world, so that it can be used to plan steps.
    def getWorldPositionAnchor(self):
        itrans = self.body.getInitialTranslation();
        irot = self.body.getInitialRotation();
        return irot * self.displacement + itrans;
        
    def getRootPosition(self):
        itrans = self.body.getTranslation();
        irot = self.body.getRotation();
        return irot * self.displacement + itrans, irot;
    
    def getSkeletonPosition(self):
        # orp, orr = self.body.getTranslation(), self.body.getRotation();
        rtp, rtr = self.getRootPosition();
        return ([rtp],[rtr]);
    
        