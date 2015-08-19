from model import HexapodBody, HexapodLeg;
from visualize import Visualizer;
import numpy as np;

def run():
    body = HexapodBody();
    legs = [HexapodLeg(body, legid) for legid in ["front left", "front right", "middle left", "middle right", "rear left", "rear right"]];
    
    iktest(body, legs);
    #animate(body, legs);
    
def iktest(body, legs):
    viz = Visualizer(body, legs);
    
    body.setRotation(-0.2, 0.1, 0.0);
    alpha = 0.02;
    
    while True:
        for l in legs:
            pos, jacob_inv = l.computeInverseKinematicsPass();
            ee = pos[-1];
            # Calculate the difference to target:
            delta = np.matrix([[0], [0], [ee[2]]]);
            update = (jacob_inv * delta * alpha).tolist();
            for u,s in zip(update, l.getSegments()):
                s.setRotation(s.getRotation() - u[0]);
            
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