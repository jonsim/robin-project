SRC1=Common.cpp
SRC2=SerialInterface.cpp
SRC3=Robot.cpp
SRC4=Vision.cpp
SRC5=TCPInterface.cpp
EXE=Robot
OBJ1=Release/Common.o
OBJ2=Release/SerialInterface.o
OBJ3=Release/Robot.o
OBJ4=Release/Vision.o
OBJ5=Release/TCPInterface.o
DEF=Release/Robot.d
DEF4=Release/Vision.d

MAKE_FLAGS=-MD -MP -MT "$(DEF4) $(OBJ4)" -c
     ^^^^^    OMG THIS IS BROKEN^^^^^^^^^^^^^
ARCH_FLAGS=-march=armv7-a -mtune=cortex-a8 -mfpu=neon -fno-tree-pre -fno-strict-aliasing -ftree-vectorize -ffast-math -funsafe-math-optimizations -fsingle-precision-constant -DNDEBUG
OPENNI_INC_FLAGS=-I/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Include -I/usr/include/ni
OPENNI_LNK_FLAGS=-L/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Lib -L./Libs -lOpenNI
OPENCV_INC_FLAGS= `pkg-config --cflags opencv` `pkg-config --libs opencv`
OPENCV_LNK_FLAGS= `pkg-config --libs opencv`
DEFINE_FLAGS=-DSERVER_ADDRESS=$JPANDA_ADDRESS -DCLIENT_ADDRESS=$JLAPTOP_ADDRESS
CC_FLAGS=-O3 -Wall -pedantic

.PHONY: all clean

all: $(EXE)

clean:
	rm $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(DEF) $(EXE)

$(EXE): $(SRC1) $(SRC2) $(SRC3) $(SRC4) $(SRC5)
	g++ $(SRC1) $(MAKE_FLAGS) $(ARCH_FLAGS) -o $(OBJ1)
	g++ $(SRC2) $(MAKE_FLAGS) $(ARCH_FLAGS) -o $(OBJ2)
	g++ $(SRC3) $(MAKE_FLAGS) $(ARCH_FLAGS) -o $(OBJ3)
	g++ $(SRC4) $(MAKE_FLAGS) $(ARCH_FLAGS) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) -o $(OBJ4)
	g++ $(SRC5) $(MAKE_FLAGS) $(ARCH_FLAGS) -o $(OBJ5)
	g++ $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OPENCV_LNK_FLAGS) $(OPENNI_LNK_FLAGS) -o $(EXE) 
