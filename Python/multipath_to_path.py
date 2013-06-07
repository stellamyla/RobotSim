"""This script converts a multipath to a piecewise linear timed trajectory.
Usage: python multipath_to_path.py [inpath.xml] [outtraj].

To tweak the speeds, change the conversion settings in the script.
"""

import math
import sys
import xml.etree.ElementTree as ET

########### CONVERSION SETTINGS ############

#turn this off if you just want an untimed milestone path 
assignTime = True
#number of configs per second
speed = 50
#if true, assigns times according to a smooth acceleration profile
accel = True
#time for pausing between sections
pauseTime = 0.25
#true if you should write out time between milestones, false if absolute time is wanted
relativeTime = False
#skip some number of milestones
#skipcount = 100
skipcount = 0



#begin code

times = []
configs = []

tree = ET.parse(sys.argv[1])
root = tree.getroot()
for sec in root.findall('section'):
    milestones = sec.findall('milestone')
    for i,m in enumerate(milestones):
        configs.append(m.attrib['config'])
        if 'time' in m.attrib:
            times.append(m.attrib['time'])
            assignTime = False
        if assignTime:
            if accel:
                n = len(milestones)+1
                T = float(n)/speed
                #total time: n/speed
                #accelerate and decelerate to use up that time
                #s(t) is such that x'(t(s)) = x'(s) s'(t) is linear inc. in the
                # first half, linearly dec. in the second.
                #x'(t(s)) = c*t(s) = s'(t) => s'(t) = ct =>
                #   s(t) = d+1/2ct^2
                #   t(s) = sqrt(2s/c)
                #we want to find t(s+1)-t(s)
                t1 = 0
                t2 = 0
                if i*2 < n:
                    t1 = T/2*math.sqrt(2*float(i)/n);
                else:
                    t1 = T-T/2*math.sqrt(2*float(n-i)/n);
                if (i+1)*2 < n:
                    t2 = T/2*math.sqrt(2*float(i+1)/n);
                else:
                    t2 = T-T/2*math.sqrt(2*float(n-i-1)/n);
                if(t1 == t2):
                    print i,t1,t2,T,n
                assert(t1 != t2)
                times.append(t2-t1)
            else:
                times.append(1.0/speed)
    if assignTime and pauseTime > 0:
        times.append(pauseTime)
        configs.append(configs[-1])

out = open(sys.argv[2],'w')
if skipcount != 0:
    print "Skipping",skipcount,"milestones at start"
if len(times) != 0:
    print "Writing",len(configs)-skipcount,"milestones to timed path",sys.argv[2]
    assert(len(times)==len(configs))
    t = 0
    for (dt,config) in zip(times,configs)[skipcount:]:
        if relativeTime:
            out.write(str(dt)+"\t"+config+'\n')
        else:
            out.write(str(t)+"\t"+config+'\n')
        t += float(dt)
else:
    print "Writing",len(configs)-skipcount,"milestones to untimed path",sys.argv[2]
    for config in configs[skipcount:]:
        out.write(config+'\n')
out.close()
