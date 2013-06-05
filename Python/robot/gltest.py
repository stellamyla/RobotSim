from robot import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from geometry import camera
from geometry import se3
from geometry import so3
from geometry import vectorops
import math
import time

view = camera.orbit()
view.dist = 4.0
w = 640

def prepare_GL():
    """Prepare drawing.
    """

    # Viewport
    glViewport(0,0,640,480)

    # Initialize
    glClearColor(0.8,0.8,0.9,0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_FLAT)

    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective (45,1.3333,0.2,20)

    # Initialize ModelView matrix
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    
    # View transformation
    mat = se3.homogeneous(view.matrix())
    cols = zip(*mat)
    pack = sum((list(c) for c in cols),[])
    glMultMatrixf(pack)

    # Light source
    glLightfv(GL_LIGHT0,GL_POSITION,[0,-1,2,0])
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT0)


ttotal = 0
fps = 50
dt = 1.0/fps
running = True
counter = 0
lasttime = time.time()
lastx = 0
lasty = 0
modifiers = 0

def startGLTest(world,sim):
    # Initialize Glut
    glutInit ([])
    # Open a window
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE)

    x = 0
    y = 0
    width = 640
    height = 480
    glutInitWindowPosition (x, y);
    glutInitWindowSize (width, height);
    glutCreateWindow ("Test")
    
    # keyboard callback
    def _keyfunc (c, x, y):
        sys.exit (0)

    glutKeyboardFunc (_keyfunc)

    def _motionfunc(x,y):
        global lastx
        global lasty
        global view
        global modifiers
        dx = x - lastx
        dy = y - lasty
        if modifiers & GLUT_ACTIVE_CTRL:
            R,t = view.matrix()
            delta = so3.apply(so3.inv(R),[float(dx)/w,-float(dy)/w,0])
            view.tgt = vectorops.add(view.tgt,delta)
        elif modifiers & GLUT_ACTIVE_SHIFT:
            view.dist *= math.exp(dy*0.01)
        else:
            view.rot[2] += float(dx)*0.01
            view.rot[1] += float(dy)*0.01        
        lastx = x
        lasty = y
    
    glutMotionFunc(_motionfunc)

    def _mousefunc(button,state,x,y):
        global lastx
        global lasty
        global modifiers
        modifiers = glutGetModifiers()
        lastx = x
        lasty = y
    glutMouseFunc(_mousefunc)

    # draw callback
    def _drawfunc ():
        # Draw the scene
        prepare_GL()
        sim.updateWorld()
        world.drawGL()
        glutSwapBuffers ()

    glutDisplayFunc (_drawfunc)

    # idle callback
    def _idlefunc ():
        global ttotal, dt, counter, lasttime

        t = dt - (time.time() - lasttime)
        if (t > 0):
            time.sleep(t)
        
        ttotal += dt
        counter += 1

        #do something random
        if ttotal > 2.0:
            print "Setting milestone"
            q = sim.getController(0).getCommandedConfig()
            print q
            q[2] += 0.5
            sim.getController(0).setMilestone(q)
            ttotal = 0.0

        # Simulation step
        sim.simulate(dt)

        lasttime = time.time()

        glutPostRedisplay()

    glutIdleFunc (_idlefunc)
    
    glutMainLoop ()


if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("robot/tx90blocks.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    sim = Simulator(world)
    startGLTest(world,sim)

