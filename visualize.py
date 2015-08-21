# Hexapod Visualizer
#
# Gaurav Manek
import matplotlib.pyplot as plt;
from mpl_toolkits.mplot3d import Axes3D;
import math;

class Visualizer(object):
    def __init__(self, hexa):
        self.body = hexa.body;
        self.legs = hexa.legs;

        self.color = ["#000000", "#F58723", "#2B4E72", "#94BA65"];
        
        plt.close("all");
        self.fig = plt.figure();
        plt.ion();
        self.ax = self.fig.add_subplot(111, projection='3d');
        plt.show();

    def coordsToPlot(self, *args):
        return [[a[i] for a in args] for i in range(3)];
        
    def update(self):
        # Check to see if its still open:
        if not plt.fignum_exists(self.fig.number):
            raise KeyboardInterrupt("Window Closed");
        
        self.ax.cla();
        self.ax.set_xlim(-100, 100);
        self.ax.set_ylim(-100, 100);
        self.ax.set_zlim(-0, 100);
           
        for lg in self.legs:
            pos, rot, joint_axis = lg.getEndEffector().computeForwardKinematics();
            prevPos = self.body.getTranslation();

            for i,currPos in enumerate(pos):
                self.ax.plot(*self.coordsToPlot(prevPos, currPos), color=self.color[i]);
                prevPos = currPos;
            
            for p, r in zip(pos[:-1], joint_axis):
                self.ax.plot(*self.coordsToPlot(p, p + r*p/math.sqrt(p | p)*20), color="#FF0099");
                
            # Draw the body, from the origin to each of the legs:
            # self.ax.plot(*self.coordsToPlot(self.body.getTranslation(), lg.getRootPosition()), color=self.body_color);
                                
        self.fig.canvas.draw();
        plt.pause(0.0001);