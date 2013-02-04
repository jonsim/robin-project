SRC1=Robot.cpp
SRC2=SerialInterface.cpp
EXE=Robot
CC_FLAGS=-O3 -Wall -pedantic

all: $(EXE)

$(EXE): $(SRC1) $(SRC2)
	g++ $(SRC1) $(SRC2) $(CC_FLAGS) -o $(EXE)

clean:
	rm $(EXE)
