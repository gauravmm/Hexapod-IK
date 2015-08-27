# Hexapod Visualizer
#
# Gaurav Manek
import matplotlib.pyplot as plt;
from mpl_toolkits.mplot3d import Axes3D;
from pyrr import Vector3;
import math;

class Visualizer(object):
    def __init__(self, hexa):
        self.hexa = hexa;
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
        
    def tick(self):
        # Check to see if its still open:
        if not plt.fignum_exists(self.fig.number):
            raise KeyboardInterrupt("Window Closed");
        
        (hx, hy, hz) = self.hexa.ref.getTranslation();
        self.ax.cla();
        self.ax.set_xlim(hx - 100, hx + 100);
        self.ax.set_ylim(hy - 100, hy + 100);
        self.ax.set_zlim(hz +   0, hz + 100);
        
        # Forward pointing:
        bt = self.body.ref.getTranslation();
        br = self.body.ref.project(Vector3([0.,70.,0.]));

        self.ax.plot(*self.coordsToPlot(bt, br), color=self.color[0]);
        self.ax.scatter(*(self.coordsToPlot(br) + ['o']), color=self.color[0]);
          
        for lg in self.legs:
            pos, rot, joint_axis = lg.getEndEffector().computeForwardKinematics();
            prevPos = self.body.ref.getTranslation();

            for i,currPos in enumerate(pos):
                self.ax.plot(*self.coordsToPlot(prevPos, currPos), color=self.color[i]);
                prevPos = currPos;
            
            for p, r in zip(pos[:-1], joint_axis):
                self.ax.plot(*self.coordsToPlot(p, p + r*10), color="#FF0099");
                
            # Draw the body, from the origin to each of the legs:
            # self.ax.plot(*self.coordsToPlot(self.body.getTranslation(), lg.getRootPosition()), color=self.body_color);
                                
        self.fig.canvas.draw();
        plt.pause(0.01);