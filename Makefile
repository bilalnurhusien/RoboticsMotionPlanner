SHELL = /bin/sh
SRC = src
OBJS =  ${SRC}/main.o ${SRC}/Helpers.o ${SRC}/Minkowski.o ${SRC}/CollisionDetector.o ${SRC}/MotionPlanning.o
CXXFLAGS = -std=c++14
CXX = g++
LIBS =  -lsfml-graphics -lsfml-window -lsfml-system

pathplanning:${OBJS}
	${CXX} ${CXXFLAGS} -o $@ ${OBJS} ${LIBS}

clean:
	-rm -f ${SRC}/*.o pathplanning
