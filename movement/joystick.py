import pygame;

joystickTarget = "T.Flight Hotas X";
joystickConfig = {
    "forward" : {"axis": 1, "multiplier":-1., "deadzone":0.001},
    "right" : {"axis": 0, "multiplier":1., "deadzone":0.001},
    "clockwise" : {"axis": 3, "multiplier":1., "deadzone":0.001},
    "body_translation" : {"toggle": 2, "scale":18., "map":[1, 0, 2], "multiplier": [1., 1., 1.]},
    "body_rotation" : {"toggle": 0, "scale":.2, "map":[1, 0, 2], "multiplier": [1., -1., -1.]}
}

class ControlSource(object):
    def __init__(self, hexa):
        self.hexa = hexa;
        pygame.init();
        pygame.joystick.init();
        
        self.joystick = None;
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i);
            joystick.init();
            if joystick.get_name() == joystickTarget:
                self.joystick = joystick;
                break;
            else:
                joystick.quit();
            
        if not self.joystick:
            raise ValueError("Can't find the joystick");

        if self.joystick:
            print "Connected to " + joystick.get_name();
        
        self.bodyTrans = [.0, .0, .0];
        self.bodyRot = [.0, .0, .0];
        self.walk = [.0, .0, .0];
    
    def __enter__(self):
        return self;
    
    def __exit__(self, a, b, c):
        self.joystick.quit();

    def process(self, cfg, vals):
        vals = [cfg["scale"] * v for v in vals];
        vals = [vals[cfg["map"][i]] for i in range(len(vals))]
        vals = [v * m for v,m in zip(vals, cfg["multiplier"])];
        return vals;
            
    def tick(self):
        # Pump events so that we can 
        pygame.event.pump();
        vals_t = ["forward", "right", "clockwise"];
        vals = [self.joystick.get_axis(joystickConfig[v]["axis"]) * joystickConfig[v]["multiplier"] for v in vals_t];
        vals = [0. if abs(v) < joystickConfig[t]["deadzone"] else v for v,t in zip(vals, vals_t)];
        
        if self.joystick.get_button(joystickConfig["body_translation"]["toggle"]):
            self.bodyTrans = self.process(joystickConfig["body_translation"], vals);
        elif self.joystick.get_button(joystickConfig["body_rotation"]["toggle"]):
            self.bodyRot = self.process(joystickConfig["body_rotation"], vals);
        else:
            self.walk = vals;
        
        #print self.walk, self.bodyRot, self.bodyTrans;

        self.hexa.setWalking(*self.walk);
        self.hexa.setBodyPose(self.bodyRot, self.bodyTrans);
    