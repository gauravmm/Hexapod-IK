# -*- coding: utf-8 -*-
"""
Configuration of this particular Hexapod.

@author: GauravManek
"""

from pyrr import Vector3, Quaternion;
import math;

class HexapodConfig(object):
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
        return ["front left", "front right", "middle left", "middle right", "rear left", "rear right"];
    
    def getLegSegmentConfig(self, leg):
        ymult = 1;
        change_angle = lambda x: x;
        angle_map = lambda x: x;
        if "left" in leg:
            ymult = -1;
            change_angle = lambda x: (math.pi - x);
            angle_map = lambda x: -x;
            
        #[rotation_axis, disp, start_angle]
        legargs = [ [angle_map, Vector3([0., 0., 1.]), Vector3([13.3, ymult * 14.9, 0]), change_angle(math.pi/2)],
                    [angle_map, Vector3([1., 0., 0.]), Vector3([0, 55.4, 0]), change_angle(-math.pi/2 + math.asin(30.1/55.4))],
                    [angle_map, Vector3([1., 0., 0.]), Vector3([0, 90.0, 0]), -math.pi/2 + change_angle(-math.pi/2 - math.asin(30.1/55.4))]
                    ];
        
        return legargs;
    
    def getLegMotionPlannerParams(self):
        return {};

    def getHexapodMotionPlannerParams(self):
        return {};