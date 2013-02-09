SRC1=ProcessFrame.cpp
EXE1=ProcessFrame
SRC2=ProcessVideo.cpp
EXE2=ProcessVideo
OPENCV_FLAGS= `pkg-config --cflags opencv` `pkg-config --libs opencv`
CC_FLAGS=-O3 -Wall

all: $(EXE1) $(EXE2)

$(EXE1): $(SRC1)
	g++ $(SRC1) $(OPENCV_FLAGS) $(CC_FLAGS) -o $(EXE1)

$(EXE2): $(SRC2)
	g++ $(SRC2) $(OPENCV_FLAGS) $(CC_FLAGS) -o $(EXE2)
	
clean:
	rm $(EXE1) $(EXE2)

