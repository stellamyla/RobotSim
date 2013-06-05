SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)

CPPFLAGS = -O2 -fPIC

default: $(OBJS)

clean:
	rm *.o

%.o: %.cpp
	$(CC) $(CPPFLAGS) -c $*.cpp -o $*.o
