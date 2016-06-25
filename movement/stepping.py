"""
Stepping

@author: GauravManek
"""

from pyrr import Quaternion, Vector3;
from referenceframe import ReferenceFrame;
import math;

worldRefFrame = ReferenceFrame();

def vectorNormalizeMax(v):
    if sum(cv**2 for cv in v) ** 0.5 > 1.:
        return vectorNormalize(v);
    return Vector3(v);
    
def vectorNormalize(v):
    sv = sum(cv**2 for cv in v) ** 0.5;
    if sv > 0:
        return Vector3([cv/float(sv) for cv in v]);
    else:
        return Vector3([0., 0., 0.]);

class HexapodStepMotionPlanner(object):
    def __init__(self, config, hexa, mp, legs, step_pattern):
        params = config.getStepParams();
        #config.getLegPhase(leg.getId(), step_pattern);
        self.legmp = [LegStepMotionPlanner(params, hexa, mp, l, step_pattern[l.getId()]) for l in legs];
        self.hexa = hexa;
        self.step_pattern = step_pattern;
        
        self.distance_scale = params["radius"] * 2;
        self.angular_scale = 0.3;
        self.frames = params["frames"];
        self.started = False;
        self.step_params = [0., 0., 0.];
    
    def setStepParams(self, forward, right, clockwise):
        self.step_params = (forward, right, clockwise);
        for l in self.legmp:
            l.setStepParams(forward, right, clockwise);
    
    def start(self):
        if not self.started:
            self.started = True;
            for l in self.legmp:
                l.start();
        
    def stop(self):
        if self.started:
            self.started = False;
            for l in self.legmp:
                l.stop(); 
        
    def tick(self):
        if self.started:
            forward, right, clockwise = self.step_params;
    
            if forward != 0.0 or right != 0.0:
                oTrans = self.hexa.ref.getTranslationRaw();
                oTrans += self.hexa.ref.getRotation() * vectorNormalizeMax([right, forward, 0.]) * self.distance_scale / float(self.frames);
                self.hexa.ref.setTranslation(oTrans)
            
            if clockwise:
                oRot = [r for r in self.hexa.ref.getRotationRaw()];
                oRot[2] -= clockwise * self.angular_scale / float(self.frames);
                self.hexa.ref.setRotation(oRot);
            
        for l in self.legmp:
            l.tick();
        
        
class LegStepMotionPlanner(object):
    def __init__(self, params, hexa, mp, leg, phase_group):
        self.radius = params["radius"];
        self.mp = mp;
        self.leg = leg;
        self.hexa = hexa;
        self.step_type = 0;
        self.started = False;
        
        self.phase_group = phase_group;
        self.step_pattern = None;
        self.step_height = params["height"];
        self.fr = 0;
        self.direction = leg.getDirection();
        self.frames = params["frames"] * 2;
        self.step_ee_pos = None;
        
        # Set the leg center position:
        lx,ly,lz = self.leg.getWorldPositionAnchor().xyz;
        cx, cy = params["center"];
        cx = float(cx * self.direction);
        cy = float(cy);
        #self.center = Vector3([float(lx + cx), float(ly + cy), 0.]);
        self.center = self.leg.getWorldPositionAnchor();
        
        
        # Calculate the direction for rotation:
        # Assuming pure clockwise rotation, what angle would this particular leg
        # have to move at to turn in place?
        # As it happens, this is quite easy:
        displ = Quaternion.from_z_rotation(-math.pi/2) * leg.ref.getTranslationBase();
        rotx = displ.x;
        roty = displ.y;
        rotr = (rotx**2 + roty**2)**0.5
        self.rotation_vec = Vector3([rotx/rotr, roty/rotr, 0]);
    
    def setStepParams(self, forward, right, clockwise):
        sval = abs(forward) + abs(right) + abs(clockwise);
        
        if sval == 0:
            self.step_pattern = None;
        else:
            # Add this, so we can switch to it at the next opportunity.
            self.step_pattern = (forward, right, clockwise);

    # Calculate a good position for the start of each step.    
    def heuristicStep(self, (forward, right, clockwise)):
        # Calculate the offset
        offset = vectorNormalize(Vector3([right, forward, 0.]) - clockwise*self.rotation_vec);
        return (self.center + self.radius * offset);
    
    def start(self):
        # We set the current phase to the starting phase:
        self.started = True;
        self.mp.updateTarget({self.leg.getId(): {"schedule":"snapto", "target":self.center, "ref":self.leg.world_ref, "frames": 1}});
        self.mp.tick();
        self.fr = round(self.phase_group * self.frames);
        
    def stop(self):
        self.started = False;
        #self.fr = -1;
    
    def tick(self):
        # If we are not moving, check if we need to begin:
        if self.fr < 0 or self.step_pattern is None:
            return; 
            # We don't need to start, return doing nothing.
        
        # Yep, we still need keyframe based animation.
        ee_pos = self.leg.getEndEffectorPosition();
        if self.fr < round(self.frames/2):
            if self.step_ee_pos is None:
                self.step_ee_pos = Vector3([ee_pos[0], ee_pos[1], 0.0]);
            self.mp.updateTarget({self.leg.getId(): {"schedule":"snapto", "target":self.step_ee_pos, "ref":worldRefFrame, "frames": 1}});
        elif self.fr == round(self.frames/2):
            self.step_ee_pos = None;
            ee_pos = ee_pos + Vector3([0., 0., self.step_height]);
            self.mp.updateTarget({self.leg.getId(): {"schedule":"snapto", "target":ee_pos, "ref":worldRefFrame, "frames": 1}});
        elif self.fr > round(self.frames/2):
            self.step_ee_pos = None;
            ee_pos = self.heuristicStep(self.step_pattern) + Vector3([0., 0., self.step_height]);
            self.mp.updateTarget({self.leg.getId(): {"schedule":"linear", "target":ee_pos, "ref":self.leg.world_ref, "frames": self.frames - self.fr}});
        else:
            print "Unreachable state";

        if not self.started and self.fr == 0:
            self.fr = -1;
        
        # Advance the clock:
        self.fr += 1;
        if self.fr == self.frames:
            self.fr = 0;
