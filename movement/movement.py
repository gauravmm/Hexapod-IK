# -*- coding: utf-8 -*-
"""
Created on Wed Aug 19 22:12:49 2015

@author: GauravManek
"""

from pyrr import Vector3, Quaternion;
import numpy as np;
import math;

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

class StepMotionPlanner(object):
    def __init__(self, config, mp, leg):
        self.params = config.getStepParams();
        self.mp = mp;
        self.leg = leg;
        self.legId = leg.getId();
        
        self.phase_group = config.getLegPhase(leg.getId());
        self.step_pattern = None;
        self.step_pattern_new = None;
        self.fr = 0;
        
        self.direction = leg.getDirection();
        self.frames = (self.params["frame_intermediate"]*2 + 4) * 2;
        # Phase numbers where the parameters may be changed:
        # This is when the robot is at the middle of the step.
        mid_phase = 1 + self.params["frame_intermediate"] + 1;
        self.phase_change_elligible = [mid_phase, self.frames/2 + mid_phase];
        # We always begin one before the starting point, just so we can schedule
        # the movement to the starting position.
        self.phase_start = self.phase_change_elligible[self.phase_group] - 1;
        if self.phase_start < 0:
            self.phase_start += self.frames;
         
        # Set the leg center position:
        lx,ly,lz = self.leg.getWorldPositionAnchor().xyz;
        cx, cy = self.params["center"];
        cx = float(cx * self.direction);
        cy = float(cy);
        self.center = Vector3([float(lx + cx), float(ly + cy), 0.])
        
        # Calculate the direction for rotation:
        # Assuming pure clockwise rotation, what angle would this particular leg
        # have to move at to turn in place?
        # As it happens, this is quite easy:
        displ = (Quaternion.from_z_rotation(-math.pi/2) * leg.getDisplacement());
        rotx = displ.x;
        roty = displ.y;
        rotr = (rotx**2 + roty**2)**0.5
        self.rotation_dirs = (rotx/rotr, roty/rotr);        
        
    def computeKeyframes(self, forward, right, clockwise):
        # Begin with figuring out the frames at which the leg is at the center:
        p = self.params;
        cx, cy = p["center"];
        cx = float(cx * self.direction);
        cy = float(cy);
        r = p["radius"];
        center = self.center;
        raise_small = [0., 0., float(p["height"])];
        raise_large = [0., 0., float(p["height_peak"])];
        rot_x, rot_y = self.rotation_dirs;
        
        rv = {};
        # 90 degrees phase:
        rv[self.phase_change_elligible[0]] = center;
        # 270 degree phase:
        rv[self.phase_change_elligible[1]] = center + raise_large;
        
        # Calculate the offset at 0 degrees, just before the leg sweeps back:
        offset = Vector3([(right + rot_x*clockwise), (-forward + rot_y*clockwise), 0]);
        offset_sum = sum(o**2 for o in offset) ** 0.5;
        offset *= r/offset_sum;
        
        # 0 degrees
        rv[0] = center + offset;
        
        # 180 degrees
        rv[self.frames/2] = center - offset;
        
        # Now append the steps where the leg leaves the surface and touches down:
        # 180 degrees + 1 frame
        rv[self.frames/2 + 1] = center - offset + raise_small;
        
        # 360 degrees - 1 frame
        rv[self.frames - 1] = center + offset + raise_small;
        
        # For correct wrapping behaviour, we append the minimum value to the
        # end, after adding the number of frames to it:
        min_k = min(rv);
        rv[min_k + self.frames] = rv[min_k];
        
        return rv;
        
    def setStepParams(self, forward, right, clockwise):
        sval = abs(forward) + abs(right) + abs(clockwise);
        
        if sval == 0:
            self.step_pattern_new = None;
        else:
            # Add this, so we can switch to it at the next opportunity.
            self.step_pattern_new = self.computeKeyframes(forward, right, clockwise);
            
    def stop(self):
        self.step_pattern_new = "stop";
    
    def tick(self):
        # If we are not moving, check if we need to begin:
        if not self.step_pattern:
            # We aren't already walking. Check if we need to start:
            if not self.step_pattern_new:
                return; # We don't need to start, return doing nothing.
            
            # We set the current phase to the starting phase:
            self.fr = self.phase_start;
            self.step_pattern = self.step_pattern_new;
            self.step_pattern_new = None;
            # The rest of the code will handle the movement.
        else:
            # Check if we can change phase here:
            if self.fr in self.phase_change_elligible:
                if self.step_pattern_new:
                    if self.step_pattern_new == "stop":
                        self.mp.updateTarget({self.legId: {"target": self.center, "frames": 1}});
                        self.step_pattern = None;
                    else:
                        self.step_pattern = self.step_pattern_new;
                    self.step_pattern_new = None;
            
        # Now we check if the current animation is over:
        if not self.mp.hasPendingMovement(self.legId):
            # If it is, we find the next animation, and the number of frames.
            # We are guaranteed to find a frame, because we did the wrapping trick.
            next_k = min(k for k in self.step_pattern.iterkeys() if k > self.fr);
            self.mp.updateTarget({self.legId: {"target": self.step_pattern[next_k], "frames": next_k - self.fr}});
            
        # Advance the clock:
        self.fr += 1;
        if self.fr == self.frames:
            self.fr = 0;

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
        self.checkAngle = lambda start, end: sum(abs(s - t) for s,t in zip(start, end)) < 0.05;
        self.getAngleMovement = {};
        self.getAngleMovement["linear"] = lambda start, end, frames: [(e-s)*1./float(frames) + s for s,e in zip(start, end)];
        self.getMovement = {};
        self.getMovement["linear"] = lambda start, end, frames: (end - start) * (1./float(frames)) + start;
        self.f = 0;
        
        # Supported "schedule"s are "linear"
        self.target = dict((l.getId(), {"target": l.getEndEffectorPosition(), "schedule": "linear", "frames": 0}) for l in legs);
        
        self.target["body"] = {
                "trans": self.body.getTranslationRaw(),
                "rot": self.body.getRotationRaw(),
                "schedule": "linear",
                "frames": 0
                };
        
    def updateTarget(self, update):
        for t in self.target:
            if t in update:
                for q in self.target[t]:
                    if q in update[t]:
                        self.target[t][q] = update[t][q];
    
    def hasPendingMovement(self, legid):
        return self.target[legid]["frames"] > 0;
    
    def tick(self):
        # Advance the motion by one step:
        
        t = self.target;
        force_legs_update = False;
        # First do the body:
        if t["body"]["frames"] > 0:
            # Check if the translation needs moving:
            if self.checkTarget(self.body.getTranslationRaw(), t["body"]["trans"]):
                # Snap to, if within range
                self.body.setTranslation(t["body"]["trans"]);
            else:
                self.body.setTranslation(self.getMovement[t["body"]["schedule"]](self.body.getTranslationRaw(), t["body"]["trans"], t["body"]["frames"]));
                force_legs_update = True; # The body has moved, we need to force the legs to move;
        
            # Check if the rotation needs moving:
            if self.checkAngle(self.body.getRotationRaw(), t["body"]["rot"]):
                # Snap to, if within range
                self.body.setRotation(*t["body"]["rot"]);
            else:
                self.body.setRotation(*self.getAngleMovement[t["body"]["schedule"]](self.body.getRotationRaw(), t["body"]["rot"], t["body"]["frames"]));
                force_legs_update = True;
                        
            t["body"]["frames"] -= 1;
            
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