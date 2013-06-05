#!/usr/bin/python

import sys
from robot import *
from robot.map import set_dict
from geometry import vectorops
from geometry.glprogram import *

class GLDebug(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"GL Debugger")
        self.world = world
        self.debug_items = []
        self.current_item = -1
        self.current_extras = []

    def load(self,fn):
        f = open(fn,'r')
        for line in f.readlines():
            value = eval(line,globals())
            self.debug_items.append(value)

    def display(self):
        self.world.drawGL()

        glDisable(GL_DEPTH_TEST)
        tagoffset = so3.apply(so3.inv(self.view.matrix()[0]),[1,-1,0.])
        for extra in self.current_extras:
            name = extra[0]
            primitive = extra[1]
            glDisable(GL_LIGHTING)
            if len(primitive)==3:
                #interpret what the item might be
                try:
                    #point
                    x,y,z = float(primitive[0]),float(primitive[1]),float(primitive[2])
                    #draw point
                    glColor3f(1,1,1)
                    glPointSize(5)
                    glBegin(GL_POINTS)
                    glVertex3fv(primitive)
                    glEnd()
                    glColor3f(0,0,0)
                    depth = se3.apply(self.view.matrix(),primitive)[2]
                    dofs = 10.0/float(self.width)
                    glRasterPos3f(*(vectorops.madd(primitive,tagoffset,depth*dofs)))
                    for c in name:
                        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,ord(c))
                except:
                    continue
            elif len(primitive)==2:
                try:
                    #rigid transform
                    R,t = primitive
                    if len(R)!=9 or len(t)!=3:
                        print "Incorrect sizes",len(R),len(t)
                        raise RuntimeError("")
                    #draw transform widget
                    length = 0.1
                    glBegin(GL_LINES)
                    glColor3f(1,1,1)
                    glVertex3fv(t)
                    glColor3f(1,0,0)
                    glVertex3fv(vectorops.madd(t,R[0:3],length))
                    glColor3f(1,1,1)
                    glVertex3fv(t)
                    glColor3f(0,1,0)
                    glVertex3fv(vectorops.madd(t,R[3:6],length))
                    glColor3f(1,1,1)
                    glVertex3fv(t)
                    glColor3f(0,0,1)
                    glVertex3fv(vectorops.madd(t,R[6:9],length))
                    glEnd()
                    glColor3f(0,0,0)
                    depth = se3.apply(self.view.matrix(),t)[2]
                    dofs = 10.0/float(self.width)
                    glRasterPos3f(*(vectorops.madd(t,tagoffset,depth*dofs)))
                    for c in name:
                        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12,ord(c))
                except:
                    raise
                    raise RuntimeError("Unknown item "+str(primitive))
            else:
                pass

    def specialfunc(self,c,x,y):
        print c

    def setItem(self,item):
        if item >= len(self.debug_items) or item < 0:
            self.current_item = -1
        else:
            self.current_item = item
        
        self.current_extras = []
        if self.current_item >= 0:
            item = self.debug_items[self.current_item]
            if isinstance(item,dict):
                if 'extras' in item:
                    self.current_extras = item['extras']
                    item = item.copy()
                    del item['extras']
                    set_dict(self.world,item)
                else:
                    set_dict(self.world,item)
            elif hasattr(item,'__iter__'):
                if len(item)==2 and isinstance(item[0],str):
                    self.current_extras.append(item)
                else:
                    self.current_extras = item
            else:
                raise RuntimeError("Unknown item "+item)                

    def keyboardfunc(self,c,x,y):
        if c == '.':
            self.current_item += 1
            if self.current_item >= len(self.debug_items):
                self.current_item = 0
            self.setItem(self.current_item)
            glutPostRedisplay()
        elif c == ',':
            self.current_item -= 1
            if self.current_item < 0:
                self.current_item = len(self.debug_items)-1
            self.setItem(self.current_item)
            glutPostRedisplay()
        else:
            print c

if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile(sys.argv[1])
    if not res:
        raise RuntimeError("Unable to load world model "+sys.argv[1])
    debugger = GLDebug(world)
    for arg in sys.argv[2:]:
        debugger.load(arg)
    debugger.setItem(0)
    debugger.run()
