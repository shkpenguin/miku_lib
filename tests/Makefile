CXX=g++
CXXFLAGS=-std=c++17 -Wall -Wextra -Iinclude -I../include -I../include/miku

SRC_DIR=../src/miku
SOURCES=$(SRC_DIR)/pid.cpp $(SRC_DIR)/lut.cpp

TEST_SOURCES=main.cpp test_pid.cpp test_geometry.cpp test_lut.cpp test_util.cpp

TARGET=run_tests

all: $(TARGET)

$(TARGET): $(SOURCES) $(TEST_SOURCES)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCES) $(TEST_SOURCES)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)
