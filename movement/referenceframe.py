from pyrr import Quaternion, Vector3;

class ReferenceFrame(object):
    def __init__(self, base_trans=None, base_rotate=None, parent=None):
        if base_trans is None:
            base_trans = Vector3([0., 0., 0.]);
        if base_rotate is None:
            base_rotate = Quaternion.from_eulers([0., 0., 0.]);
        
        if type(base_trans) is not Vector3:
            raise TypeError("Oops, trans is not a vector.");
        if type(base_rotate) is not Quaternion:
            raise TypeError("Oops, rotate is not a vector.");
        if parent is not None and type(parent) is not ReferenceFrame:
            raise TypeError("parent is not a ReferenceFrame.");
            
        self.parent = parent;
        self.base_trans = base_trans;
        self.base_rotate = base_rotate;
        self.setTranslation([0., 0., 0.]);
        self.setRotation([0., 0., 0.]);
        
    def __repr__(self):
        return "ReferenceFrame(%s, %s, %s)" % (self.getTranslation(), self.getRotation(), self.parent);
    
    def project(self, vec):
        return self.getTranslation() + (self.getRotation() * vec);
    def projectInverse(self, vec):
        return (~self.getRotation()) * (vec - self.getTranslation());
    
    def setTranslation(self, trans):
        self.trans_raw = Vector3(trans);
        self.trans = self.base_trans + Vector3(trans);    
    def setRotation(self, rotate):
        self.rotate_raw = rotate;
        self.rotate = self.base_rotate * Quaternion.from_eulers(rotate);
    def getRotationRaw(self):
        return self.rotate_raw;
    def getRotation(self):
        if self.parent:
            return self.parent.getRotation() * self.rotate;
        return self.rotate;
    def getTranslationRaw(self):
        return self.trans_raw;
    def getTranslation(self):
        if self.parent:
            return self.parent.project(self.trans);
        return self.trans;
    def getTranslationBase(self):
        return self.base_trans;

"""     
w = ReferenceFrame();
q = ReferenceFrame(Vector3([10., 0., 0.]), None, w);
print q.getTranslation();
w.setRotation([0, 0, 3.1415927*3/2]);
print q.getTranslation();
w.setRotation([0, 0, 3.1415927/2]);
w.setTranslation([0., 0., 10.])
print q.getTranslation();
"""