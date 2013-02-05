SRC1=Common.cpp
SRC2=SerialInterface.cpp
SRC3=Robot.cpp
SRC4=Vision.cpp
EXE=Robot
CC_FLAGS=-O3 -Wall -pedantic

all: $(EXE)

$(EXE): $(SRC1) $(SRC2) $(SRC3) $(SRC4)
	g++ $(SRC1) $(SRC2) $(SRC3) $(SRC4) $(CC_FLAGS) -o $(EXE)

clean:
	rm $(EXE)
