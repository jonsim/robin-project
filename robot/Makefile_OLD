SRC1=Common.cpp
SRC2=SerialInterface.cpp
SRC3=Robot.cpp
SRC4=Vision.cpp
SRC5=TCPInterface.cpp
EXE=Robot
CC_FLAGS=-O3 -Wall -pedantic
DEFINE_FLAGS=-DSELF_ADDRESS=$JPANDA_ADDRESS -DTARGET_ADDRESS=$JLAPTOP_ADDRESS

all: $(EXE)

$(EXE): $(SRC1) $(SRC2) $(SRC3) $(SRC4) $(SRC5)
	g++ $(SRC1) $(SRC2) $(SRC3) $(SRC4) $(SRC5) $(CC_FLAGS) $(DEFINE_FLAGS) -o $(EXE)

clean:
	rm $(EXE)
