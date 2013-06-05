from robot import *
from geometry.glprogram import *
import numpy as np

class GLTest(GLRealtimeProgram):
    def __init__(self,world,sim):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.sim = sim
        #self.operationalSpaceMethod = 'transpose'
        self.operationalSpaceMethod = 'pseudoinverse'
        self.gravityCompensation = False
        self.useITerm = False
        self.iterm = [0]*12

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        rfs = sim.getController(0).getNamedSensor("RF_ForceSensor")
        print "Sensor values:",rfs.getMeasurements()
        print "Measurement names:",rfs.measurementNames()
        return
        #do something random
        #if self.ttotal > 2.0:
            #velocity control
            #print "Setting velocity"
            #dq = [0.0]*len(q)
            #dq[2] = 0.5
            #sim.getController(0).setVelocity(dq,0.5)
            #milestone control
            #print "Setting milestone"
            #q = sim.getController(0).getCommandedConfig()
            #print q
            #q[2] += 0.5
            #sim.getController(0).setMilestone(q)
            #self.ttotal = 0.0
        """
        print "Setting faked PID "
        q = sim.getController(0).getSensedConfig()[1:]
        dq = sim.getController(0).getSensedVelocity()[1:]
        qdes = [0,-0.25,0.25,0,0,0,0]
        kP = [1000,1000,500,200,50,50,10]
        kD = [800,800,50,20,5,5,1]
        t = [kpi*(qd-qi)+kdi*(0.0-dqi) for (kpi,kdi,qd,qi,dqi) in zip(kP,kD,qdes,q,dq)]
        sim.getController(0).setTorque(t)
        """
        if self.ttotal < 0.0:
            sim.getController(0).setPIDCommand([0.0,0.0,0.8,0,0,0,0],[0]*7)
        else:
            q = sim.getController(0).getSensedConfig()
            dq = sim.getController(0).getSensedVelocity()
            qcmd = sim.getController(0).getCommandedConfig()
            self.world.robot(0).setConfig(qcmd)
            p = self.world.robot(0).getLink(7).getWorldPosition([0.1,0,0])
            print "Commanded position",p
            self.world.robot(0).setConfig(q)
            J = self.world.robot(0).getLink(7).getPositionJacobian([0.1,0,0])
            p = self.world.robot(0).getLink(7).getWorldPosition([0.1,0,0])
            print "Current position",p
            pdes = [-0.7,0,0.8]
            print "Desired position",pdes

            kP = 1
            v = [kP*(pdi-pi) for (pdi,pi) in zip(pdes,p)]
            if vectorops.norm(v) > 0.1:
                v = vectorops.mul(v,0.1/vectorops.norm(v))
            print "V=",v

            #x' = J*q'
            #q' = J^T x'  (jacobian transpose control, often slow to converge)
            #q' = J^+ x'  (jacobian pseudoinverse control, much faster)
            dqdes = None
            if self.operationalSpaceMethod=='pseudoinverse':
                #jacobian pseudoinverse control
                Jinv = np.linalg.pinv(np.array(J),rcond=1e-2)
                dqdes = np.dot(Jinv,np.array(v))
                #print Jinv
            else:
                dqdes = np.dot(np.array(J).T,np.array(v))
            
            #cap at joint velocity limits (picked at 2 for no particular reason)
            for i in range(len(dqdes)):
                if abs(dqdes[i]) > 2:
                    dqdes[i] = 2*dqdes[i]/abs(dqdes[i])
            print "Predicted V:",np.dot(np.array(J),dqdes)[0:3]
            
            #Integral term adaptation
            if self.useITerm:
                self.iterm = vectorops.madd(self.iterm,vectorops.sub(q,qcmd),self.dt)
                kI = 2
                dqdes = vectorops.madd(dqdes,self.iterm,kI)
            qdes = vectorops.madd(qcmd,dqdes,self.dt)

            #gravity compensator
            if self.gravityCompensation:
                kPMotor = [1,40000,50000,20000,10000,2000,1000,50,1,1,1,1]
                kDMotor = [1,100,100,70,40,20,2,0.1,1,1,1,1]
                G = self.world.robot(0).getGravityForces((0,0,-9.8))
                #kP * (qcmd'-q) + kD * (dqcmd'-dq) = t
                #want t = -G + kP * (qcmd-q) + kD * (dqcmd-dq)
                #one solution: let dqcmd' = dqcmd
                #Solve for qcmd' = -G/kP + qcmd
                for i in range(len(qdes)):
                    qdes[i] = qdes[i] - G[i]/kPMotor[i]
                #print "G=",G
            sim.getController(0).setPIDCommand(qdes[1:8],dqdes[1:8])
        
        self.sim.simulate(self.dt)

if __name__ == "__main__":
    world = WorldModel()
    res = world.readFile("../hubo_files/hubo_plane.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    world.robot(0).setConfig(q)
    sim = Simulator(world)
    GLTest(world,sim).run()

