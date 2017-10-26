TARGET = DLite
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


QMAKE_CXXFLAGS += -std=c++11 -O2 -Wall -Wextra

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}


SOURCES += main.cpp \
    config.cpp \
    environmentoptions.cpp \
    map.cpp \
    mission.cpp \
    tinyxml2.cpp \
    xmllogger.cpp \
    dlite.cpp \
    localmap.cpp

DISTFILES +=

HEADERS += \
    config.h \
    environmentoptions.h \
    gl_const.h \
    heuristics.h \
    ilogger.h \
    map.h \
    mission.h \
    node.h \
    openlist.h \
    searchresult.h \
    tinyxml2.h \
    xmllogger.h \
    dlite.h \
    localmap.h

