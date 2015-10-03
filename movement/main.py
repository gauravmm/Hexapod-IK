from body import Hexapod;
from config import HexapodConfig;
from visualize import Visualizer;
from hardware import SerialLink;
from joystick import ControlSource;

def run():
    cfg = HexapodConfig();
    hexa = Hexapod(cfg);
    with SerialLink(cfg, hexa, "COM4") as lnk:
        controlLoop(hexa, lnk, False);

def controlLoop(hexa, lnk, viz):
    if viz:
        viz = Visualizer(hexa);
    
    with ControlSource(hexa) as control:
        while True:
            control.tick();
            hexa.tick();
            lnk.tick();
            if viz:
                viz.tick();
                
def testLoop(hexa, lnk, viz):
    if viz:
        viz = Visualizer(hexa);
    
    hexa.setWalking(1., 0., 0.);
    while True:
        hexa.tick();
        lnk.tick();
        if viz:
            viz.tick();

if __name__ == "__main__":
    try:
        run();
    except (KeyboardInterrupt, SystemExit):
        pass;