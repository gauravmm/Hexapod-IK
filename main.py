from body import Hexapod;
from config import HexapodConfig;
from visualize import Visualizer;
import numpy as np;
from pyrr import Vector3;

def run():
    hexa = Hexapod(HexapodConfig());
    
    hexatest1(hexa);
    #iktest(hexa);
    #animate(hexa.body, hexa.legs);

def hexatest1(hexa):
    viz = Visualizer(hexa);
    
    t = {};
    fr = 10;
    for l in hexa.legs:
        x,y,z = l.getWorldPositionAnchor().xyz;
        offset = 30.;
        if "left" in l.getId():
            offset *= -1.;
        t[l.getId()] = {"target": Vector3([x + offset, y, 0.]), "frames": fr};
    
    hexa.mp.updateTarget(t);
    
    while fr > 0:
        fr -= 1;
        hexa.tick();
        viz.update();

def iktest(hexa):
    viz = Visualizer(hexa);
    
    body, legs = hexa.body, hexa.legs;
    #legs = [l for l in legs if "middle right" in l.getId()];
    body.setRotation(0.0, 0.0, 0.0);
    alpha = 0.2;
    offset_x = 50;
    offset_y = 10;
    
    while True:
        for l in legs:
            d = -1 if "left" in l.getId() else 1;
            pos, jacob_inv = l.computeInverseKinematicsPass();   
            ee = pos[-1];
            x,y,z = l.getWorldPositionAnchor().xyz;
            #t = Vector3([100, 10, 0]);
            t = Vector3([x + d * offset_x, y + offset_y, 0]);
            
            # Calculate the difference to target:
            delta = np.array([[v] for v in ee - t]);
            update = (jacob_inv * delta * alpha).tolist();
            
            for u,s in zip(update, l.getSegments()):
                s.setRotation(s.getRotation() + u[0]);
            
            
        #body.setRotation(*angle);
        viz.update();

def animate(body, legs):
    viz = Visualizer(body, legs);
    
    vals = range(0, -10, -1) + range(-10, 10, 1) + range(10, 0, -1);
    anim  = [[p * 0.04, 0., 0.] for p in vals];
    anim += [[0., p * 0.04, 0.] for p in vals];
    anim += [[0., 0., p * 0.04] for p in vals];
   
    while True:
        for angle in anim:
            for l in legs:
                for i,s in enumerate(l.getSegments()):
                    s.setRotation(angle[i]);
                    
            #body.setRotation(*angle);
            viz.update();

if __name__ == "__main__":
    try:
        run();
    except (KeyboardInterrupt, SystemExit):
        pass;