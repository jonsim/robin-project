SRC1=CaptureFrame.cpp
EXE1=CaptureFrame
OBJ1=Release/CaptureFrame.o
DEF1=Release/CaptureFrame.d
SRC2=CaptureVideo.cpp
EXE2=CaptureVideo
OBJ2=Release/CaptureVideo.o
DEF2=Release/CaptureVideo.d

MAKE_FLAGS1=-MD -MP -MT "$(DEF1) $(OBJ1)" -c
MAKE_FLAGS2=-MD -MP -MT "$(DEF2) $(OBJ2)" -c
ARCH_FLAGS=-march=armv7-a -mtune=cortex-a8 -mfpu=neon -O3 -fno-tree-pre -fno-strict-aliasing -ftree-vectorize -ffast-math -funsafe-math-optimizations -fsingle-precision-constant -O2 -DNDEBUG
INCL_FLAGS=-I/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Include -I/usr/include/ni
LNKR_FLAGS=-L/home/jon/kinect/OpenNI/Platform/Linux/Redist/OpenNI-Bin-Dev-Linux-Arm-v1.5.4.0/Lib -L./Libs -lOpenNI

.PHONY: all clean

all: $(EXE1) $(EXE2)

clean:
	rm $(OBJ1) $(DEF1) $(EXE1)
	rm $(OBJ2) $(DEF2) $(EXE2)

$(EXE1): $(SRC1)
	g++ $(MAKE_FLAGS1) $(ARCH_FLAGS) $(INCL_FLAGS) -o $(OBJ1) $(SRC1)
	g++ -o $(EXE1) $(OBJ1) $(LNKR_FLAGS)

$(EXE2): $(SRC2)
	g++ $(MAKE_FLAGS2) $(ARCH_FLAGS) $(INCL_FLAGS) -o $(OBJ2) $(SRC2)
	g++ -o $(EXE2) $(OBJ2) $(LNKR_FLAGS)
