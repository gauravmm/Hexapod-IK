# -*- coding: utf-8 -*-
"""
Created on Wed Aug 19 22:12:49 2015

@author: GauravManek
"""

from pyrr import Vector3;
import numpy as np;

class LegTarget(object):
    def __init__(self, *args):
        if len(args) in [1, 2]:
            if isinstance(args[0], Vector3):
                self.pt = args[0];
            else:
                raise TypeError("Single argument is not pyrr.Vector3");
            
            if len(args) == 2:
                self.precision = args[1];
                
        elif len(args) == 3:
            self.pt = args;
        else:
            raise TypeError("Incorrect number of arguments.");
    
    def dist(self, ee):
        return ee - self.pt;
        
class LegTargetPlane(LegTarget):
    def __init__(self, pt, n, *args):
        if isinstance(pt, Vector3) and isinstance(n, Vector3):
            self.pt = pt;
            self.n = n;
        else:
            raise TypeError("Arguments are not pyrr.Vector3");
        
        if len(args) == 1:
            self.precision = args[0];
    
    def dist(self, ee):
        # Project ee onto the plane:
        ee_proj = self.pt - (((ee - self.pt) | self.n) * self.n) / (self.n | self.n);
        return ee - ee_proj;

class LegMotionPlanner(object):
    def __init__(self, leg, **kwargs):
        self.leg = leg;
        self.options = kwargs;        
        
        if "alpha" not in self.options and "schedule" not in self.options:
            self.options["alpha"] = 0.2;
        elif "alpha" in self.options and "schedule" in self.options:
            self.options["schedule"] = [self.options["alpha"] * s for s in self.options["schedule"]];
            del self.options["alpha"];
        
        if "delta" not in self.options:
            self.options["delta"] = 1; # linear distance to target in mm 
        
        self.options["delta_sq"]  = self.options["delta"] ** 2;
        del self.options["delta"];
        
        if "schedule" in self.options:
            self.options["maxiter"] = len(self.options["schedule"])-1;
        if "maxiter" not in self.options:
            self.options["maxiter"] = 300;
    
    def snapTo(self, target):
        # Clone the leg, so we operate only on the copy:
        return self.computeIK(self.leg.clone(), target);
    
    def computeIK(self, leg, target):
        i = 0;
        alpha = self.options["alpha"] if "alpha" in self.options else None;
        while True:
            if i >= self.options["maxiter"]:
                raise ValueError("Not converged to solution within maxiter.");
            
            if "schedule" in self.options:
                # We know its within bounds because maxiter is pegged to the
                # schedule.
                alpha = self.options["schedule"][i];
            
            pos, jacob_inv = leg.computeInverseKinematicsPass();
            ee = pos[-1];
            
            # Calculate the difference to target:
            delta = target.dist(ee);
            
            if (delta | delta) < self.options["delta_sq"]:
                # We have converged!
                # Return the translations to reach the target.
                return [s.getRotation() for s in leg.getSegments()];
            
            update = (jacob_inv * np.array([[delta.x], [delta.y], [delta.z]]) * alpha).tolist();
            for u,s in zip(update, leg.getSegments()):
                s.setRotation(s.getRotation() + u[0]);
            
            i += 1;
    
class HexapodMotionPlanner(object):
    def __init__(self, config, body, legs):
        self.body = body;
        self.legs = legs;
        self.leg_dict = dict((l.getId(), (l, LegMotionPlanner(l, **config.getLegMotionPlannerParams()))) for l in legs);
        
        self.options = config.getHexapodMotionPlannerParams();
        
        if "delta" not in self.options:
            self.options["delta"] = 0.1;
        
        self.options["delta_sq"] = self.options["delta"] ** 2;
        del self.options["delta"];
        
        self.checkTarget = lambda start, end: (start | end) <= self.options["delta_sq"];
        self.checkAngle = lambda start, end: sum(abs(s, t) for s,t in zip(start, end)) < 0.05;
        self.getAngleMovement = {};
        self.getAngleMovement["linear"] = lambda start, end, frames: [(e-s)*1./float(frames) + s for s,e in zip(start, end)];
        self.getMovement = {};
        self.getMovement["linear"] = lambda start, end, frames: (end - start) * (1./float(frames)) + start;
        self.f = 0;
        
        # Supported "schedule"s are "linear"
        self.target = dict((l.getId(), {"target": l.getEndEffectorPosition(), "schedule": "linear", "frames": 0}) for l in legs);
        
        self.target["body"] = {
                "trans": self.body.getTranslation(),
                "rot": self.body.getRotation(),
                "schedule": "linear",
                "frames": 0
                };
        
    def updateTarget(self, update):
        for t in self.target:
            if t in update:
                for q in self.target[t]:
                    if q in update[t]:
                        self.target[t][q] = update[t][q];
    
    def tick(self):
        # Advance the motion by one step:
        
        t = self.target;
        force_legs_update = False;
        # First do the body:
        if t["body"]["frames"] > 0:
            # Check if the translation needs moving:
            if self.checkTarget(self.body.getTranslation(), t["body"]["trans"]):
                # Snap to, if within range
                self.body.setTranslation(t["body"]["trans"]);
            else:
                self.body.setTranslation(self.getMovement[t["body"]["schedule"]](self.body.getTranslation(), t["body"]["trans"], t["body"]["frames"]));
                force_legs_update = True; # The body has moved, we need to force the legs to move;

            t["body"]["frames"] -= 1;
        
            # Check if the rotation needs moving:
            if self.checkAngle(self.body.getRotation(), t["body"]["rot"]):
                # Snap to, if within range
                self.body.setRotation(t["body"]["rot"]);
            else:
                self.body.setRotation(self.getAngleMovement[t["body"]["schedule"]](self.body.getRotation(), t["body"]["rot"], t["body"]["frames"]));
            
        # Now do all legs:
        for k in self.leg_dict:
            if t[k]["frames"] == 0 and force_legs_update:
                t[k]["frames"] = 1; # Ensure that we acquire the final target in 1 frame;
                
            # Does the leg need to be moved?
            if t[k]["frames"] > 0:
                l, lmp = self.leg_dict[k];
                # Check if it needs moving:
                ee = l.getEndEffectorPosition();
                if not self.checkTarget(ee, t[k]["target"]):
                    new_ee = self.getMovement[t[k]["schedule"]](ee, t[k]["target"], t[k]["frames"]);
                    # Solve the IK for the new position:
                    l.update(lmp.snapTo(LegTarget(new_ee)));
                
                t[k]["frames"] -= 1;
                    
        self.f += 1;