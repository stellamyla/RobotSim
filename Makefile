include Makefile.config

FLAGS = $(CPPFLAGS) $(addprefix -D, $(DEFINES))

OBJDIR = objs
LIBDIR = lib

DIRS = Modeling View Control Planning Simulation IO Contact
OBJS= $(foreach dir,$(DIRS), $(dir)/$(OBJDIR)/*.o)
LIB = $(addprefix -L, $(LIBDIRS))  $(addprefix -l, $(LIBS))
LIBROBOTSIM = -L$(LIBDIR) -lRobotSim 

##################### Start the action #########################
default: RobotTest 
.PHONY: RobotTest SimTest SimUtil PosMeasure UserTrials UserTrialsMT deps lib

deps:
	cd Library/KrisLibrary; make KrisLibrary
	cd Library/tinyxml; make 

lib:
	cd Modeling; make
	cd Contact; make 
	cd View; make
	cd Simulation; make
	cd Control; make 
	cd Planning; make
	cd IO; make
	mkdir -p $(LIBDIR)
	ar rcs $(LIBDIR)/libRobotSim.a $(foreach dir,$(DIRS),$(dir)/$(OBJDIR)/*.o)
	ranlib $(LIBDIR)/libRobotSim.a

clean:
	cd Main; make clean
	cd Modeling; make clean
	cd Contact; make clean
	cd Simulation; make clean
	cd Control; make clean
	cd Planning; make clean
	cd IO; make clean
	rm $(LIBDIR)/*.a

RobotTest: lib
	cd Main; make test.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/test.o $(LIBROBOTSIM) $(LIB) -o $@

RobotPose: lib
	cd Main; make pose.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/pose.o $(LIBROBOTSIM) $(LIB) -o $@

Cartpole:  lib
	cd Main; make cartpole.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/cartpole.o $(LIBROBOTSIM) $(LIB) -o $@

PosMeasure:  lib
	cd Main; make posmeasure.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/posmeasure.o  $(LIBROBOTSIM) $(LIB) -o $@

SimTest:  lib
	cd Main; make simtest.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/simtest.o $(LIBROBOTSIM) $(LIB) -o $@

SimUtil:  lib
	cd Main; make simutil.o
	 $(CC) $(FLAGS) Main/$(OBJDIR)/simutil.o $(LIBROBOTSIM) $(LIB) -o $@

UserTrials:  lib
	cd Main; make usertrials.o
	 $(CC) $(FLAGS) $(OBJS) Main/$(OBJDIR)/usertrials.o Input/$(OBJDIR)/*.o $(LIBROBOTSIM)  $(LIBROBOTSIM) $(LIB) -o $@

UserTrialsMT:  lib
	cd Main; make usertrials_multithread.o
	cd Input; make
	 $(CC) $(FLAGS)  Main/$(OBJDIR)/usertrials_multithread.o Input/$(OBJDIR)/*.o $(LIBROBOTSIM) $(LIB) -o $@

python: lib
	cd Python/robot; make
	cd Python/geometry; make
	cd Python/motionplanning; make
	cd Python/rootfind; make