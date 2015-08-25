from body import Hexapod;
from config import HexapodConfig;
from visualize import Visualizer;
from hardware import SerialLink;
import numpy as np;
from pyrr import Vector3;
import time;

def run():
    cfg = HexapodConfig();
    hexa = Hexapod(cfg);
    with SerialLink(cfg, hexa, "COM7") as lnk:
        hexatest2(hexa, lnk);

def hexatest2(hexa, lnk):
    viz = None;
    #viz = Visualizer(hexa);
    
    hexa.setBodyPose([0., 0.0, 0.], [20., 0., 0.]);
    hexa.setWalking(1., 0., 0.);
    while True:
        hexa.tick();
        lnk.tick();
        if viz:
            viz.tick();

def hexatest1(hexa, lnk):
    viz = Visualizer(hexa);

    while True:
        time.sleep(0.4);
        for offset in [-0.2, 0.2]:
            t = {};
            fr = 3;
            
            
            t["body"] = {"frames": fr, "trans": Vector3([0., 0. + 50*offset, 0.]), "rot": [0., 0., 0.]};
                        
            for l in hexa.legs:
                di = 1;
                x,y,z = l.getWorldPositionAnchor().xyz;
                
                if "left" in l.getId():
                    di = -1.;
                    
                t[l.getId()] = {"target": Vector3([x + di*40, y - 20, 0.]), "frames": fr};
            
            
            hexa.mp.updateTarget(t);
            
            while fr > 0:
                fr -= 1;
                hexa.tick();
                lnk.tick();
                viz.tick();
            

def iktest(hexa):
    viz = Visualizer(hexa);
    
    body, legs = hexa.body, hexa.legs;
    #legs = [l for l in legs if "middle right" in l.getId()];
    body.setRotation(0.2, 0.1, 0.0);
    alpha = 0.2;
    offset_x = 50;
    offset_y = 10;
    
    while True:
        for l in legs:
            d = -1 if "left" in l.getId() else 1;
            pos, jacob_inv = l.computeInverseKinematicsPass();   
            ee = pos[-1];
            x,y,z = l.getWorldPositionAnchor().xyz;
            t = Vector3([x + d * offset_x, y + offset_y, 0]);
            
            # Calculate the difference to target:
            delta = np.array([[v] for v in ee - t]);
            update = (jacob_inv * delta * alpha).tolist();
            
            for u,s in zip(update, l.getSegments()):
                s.setRotation(s.getRotation() + u[0]);
            
            
        viz.tick();

if __name__ == "__main__":
    try:
        run();
    except (KeyboardInterrupt, SystemExit):
        pass;