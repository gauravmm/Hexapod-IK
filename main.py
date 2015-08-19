from model import HexapodBody, HexapodLeg;
from visualize import Visualizer;


def run():
    body = HexapodBody();
    legs = [HexapodLeg(body, legid) for legid in ["front left", "front right", "middle left", "middle right", "rear left", "rear right"]];
    
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