TEMPLATE = app
CONFIG += console c++11

QMAKE_CXXFLAGS += /openmp

win32{
    INCLUDEPATH += $(OPENCV_DIR_VC14)/include
    LIBS += -L$(OPENCV_DIR_VC14)/x64/vc14/lib -lopencv_core2412d -lopencv_highgui2412d
}
else{
#    QMAKE_CXXFLAGS += -fopenmp
    LIBS += -lX11 -lpthread -lopencv_core -lopencv_highgui -liomp5
}

SOURCES += main.cpp


