#!/usr/bin/python

import sys
from robot import *
from robot import robotcollide
from geometry import vectorops
from geometry import collide
from geometry.glprogram import *

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, and
        #simulation flag
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate:
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2 and state==0:
            print [o.getName() for o in self.click_world(x,y)]
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation
        print c,"pressed"
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        self.collider.updateFrames()
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = collide.rayCast(g[1],s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]


if __name__ == "__main__":
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    viewer.run()
