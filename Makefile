SRC=CaptureFrame.cpp
EXE=CaptureFrame
OBJ=Release/CaptureFrame.o
DEF=Release/CaptureFrame.d

MAKE_FLAGS=-MD -MP -MT "$(DEF) $(OBJ)" -c
ARCH_FLAGS=-march=armv7-a -mtune=cortex-a8 -mfpu=neon -O3 -fno-tree-pre -fno-strict-aliasing -ftree-vectorize -ffast-math -funsafe-math-optimizations -fsingle-precision-constant -O2 -DNDEBUG
INCL_FLAGS=-I/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Include -I/usr/include/ni
LNKR_FLAGS=-L/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Lib -L./Libs -lOpenNI

.PHONY: all clean

all: $(EXE)

clean:
	\rm $(OBJ) $(DEF) $(EXE)

$(EXE): $(SRC)
	g++ $(MAKE_FLAGS) $(ARCH_FLAGS) $(INCL_FLAGS) -o $(OBJ) $(SRC)
	g++ -o $(EXE) $(OBJ) $(LNKR_FLAGS)
