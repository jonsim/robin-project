SRC1=Common.cpp
DEF1=Release/Common.d
OBJ1=Release/Common.o
SRC2=SerialInterface.cpp
DEF2=Release/SerialInterface.d
OBJ2=Release/SerialInterface.o
SRC3=Robot.cpp
DEF3=Release/Robot.d
OBJ3=Release/Robot.o
SRC4=Vision.cpp
DEF4=Release/Vision.d
OBJ4=Release/Vision.o
SRC5=TCPInterface.cpp
DEF5=Release/TCPInterface.d
OBJ5=Release/TCPInterface.o
SRC6=RobotClient.cpp
DEF6=Release/RobotClient.d
OBJ6=Release/RobotClient.o
SRC7=RoBin.cpp
DEF7=Release/RoBin.d
OBJ7=Release/RoBin.o
EXE1=RoBin
EXE2=RobotClient

MAKE_FLAGS1=-MD -MP -MT "$(DEF1) $(OBJ1)"
MAKE_FLAGS2=-MD -MP -MT "$(DEF2) $(OBJ2)"
MAKE_FLAGS3=-MD -MP -MT "$(DEF3) $(OBJ3)"
MAKE_FLAGS4=-MD -MP -MT "$(DEF4) $(OBJ4)"
MAKE_FLAGS5=-MD -MP -MT "$(DEF5) $(OBJ5)"
MAKE_FLAGS7=-MD -MP -MT "$(DEF7) $(OBJ7)"
ARCH_FLAGS=-march=armv7-a -mtune=cortex-a8 -mfpu=neon -fno-tree-pre -fno-strict-aliasing -ftree-vectorize -ffast-math -funsafe-math-optimizations -fsingle-precision-constant -DNDEBUG
OPENNI_INC_FLAGS=-I/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Include -I/usr/include/ni -Wno-unknown-pragmas
OPENNI_LNK_FLAGS=-L/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Lib -L./Libs -lOpenNI
OPENCV_INC_FLAGS= `pkg-config --cflags opencv`
OPENCV_LNK_FLAGS= `pkg-config --libs opencv`
DEF_FLAGS=-DSERVER_ADDRESS=`cat ~/.ip_jpanda` -DCLIENT_ADDRESS=`cat ~/.ip_jlaptop`
CC_FLAGS=-O3 -Wall

.PHONY: all clean panda laptop

all: $(EXE1)

panda: $(EXE1)

laptop: $(EXE2)

clean:
	rm -f $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OBJ7) $(DEF1) $(DEF2) $(DEF3) $(DEF4) $(DEF5) $(DEF7) $(EXE1) $(EXE2)

$(EXE1): $(SRC1) $(SRC2) $(SRC3) $(SRC4) $(SRC5) $(SRC7)
	g++ $(SRC1) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ1)
	g++ $(SRC2) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ2)
	g++ $(SRC3) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ3)
	g++ $(SRC4) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ4)
	g++ $(SRC5) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ5)
	g++ $(SRC7) -c $(ARCH_FLAGS) $(CC_FLAGS) $(MAKE_FLAGS4) $(OPENCV_INC_FLAGS) $(OPENNI_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ7)
	g++ $(OBJ1) $(OBJ2) $(OBJ3) $(OBJ4) $(OBJ5) $(OPENCV_LNK_FLAGS) $(OPENNI_LNK_FLAGS) -o $(EXE1)

$(EXE2): $(SRC5)
	g++ $(SRC1) -c $(CC_FLAGS) $(OPENCV_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ1)
	g++ $(SRC5) -c $(CC_FLAGS) $(OPENCV_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ5)
	g++ $(SRC6) -c $(CC_FLAGS) $(OPENCV_INC_FLAGS) $(DEF_FLAGS) -o $(OBJ6)
	g++ $(OBJ1) $(OBJ5) $(OBJ6) $(OPENCV_LNK_FLAGS) -o $(EXE2)

