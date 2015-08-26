# -*- coding: utf-8 -*-
"""
Configuration of this particular Hexapod.

@author: GauravManek
"""

from pyrr import Vector3;
import math;

class HexapodConfig(object):
    def __init__(self):
        lg = ["front left", "middle left", "rear left", "front right", "middle right", "rear right"];
        legPhaseAnt = dict(zip(lg, [0, 1, 0, 1, 0, 1]));
        legPhaseDiagonal = dict(zip(lg, [0, 1, 2, 0, 1, 2]));
        legPhaseMillipede = dict(zip(lg, [0, 1, 2, 3, 4, 5]));
        self.legPhase = [legPhaseAnt, legPhaseDiagonal, legPhaseMillipede];
    
    def getStepParams(self):
        return {
            "center": (50, -20),
            "radius": 17,
            "height": 5,
            "height_peak": 15,
            "frame_intermediate": 1
        }
        
    def getLegPhase(self, legid, patternid):
        return self.legPhase[patternid][legid];
    
    def getInitialPose(self):
        return [0., 0., 0.], [0., 0., 50.];
    
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
        
    def getLegs(self):
        # They will be sent to the robot in this order:
        return ["front left", "middle left", "rear left", "rear right", "middle right", "front right"];
    
    def getLegSegmentConfig(self, leg):
        ymult = 1;
        change_angle = lambda x: x;
        if "left" in leg:
            ymult = -1;
            change_angle = lambda x: (math.pi - x);
            
        #[rotation_axis, disp, start_angle]
        legargs = [ [Vector3([0., 0., 1.]), Vector3([13.3, ymult * 14.9, 0]), change_angle(math.pi/2)],
                    [Vector3([1., 0., 0.]), Vector3([0, 55.4, 0]), change_angle(-math.pi/2 + math.asin(30.1/55.4))],
                    [Vector3([1., 0., 0.]), Vector3([0, 90.0, 0]), -math.pi/2 + change_angle(-math.pi/2 - math.asin(30.1/55.4))]
                    ];
        
        return legargs;
    
    def getLegMotionPlannerParams(self):
        return {};

    def getHexapodMotionPlannerParams(self):
        return {};
    
    def rad2enc(self, rad):
        return round(rad * 160.0 / (math.pi/2));