TEMPLATE = app
CONFIG += console c++11

QMAKE_CXXFLAGS += -fopenmp

SOURCES += main.cpp

LIBS += -lX11 -lpthread -lopencv_core -lopencv_highgui -liomp5

