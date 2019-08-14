#-------------------------------------------------
#
# Project created by QtCreator 2019-01-30T15:59:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = lidar_calibration
TEMPLATE = app

CONFIG += static

QMAKE_CXXFLAGS += -std=c++14

LIBS += -lboost_system -lboost_filesystem -lboost_thread -lpcl_visualization -lpcl_common -lpcl_io -lpcl_kdtree -lpcl_search

SOURCES += main.cpp \
    mainwindow.cpp \
    addlidardialog.cpp \
    lidar_driver/driver.cpp \
    lidar_driver/input.cpp \
    lidar_driver/rawdata.cpp \
    lidar_driver/lidar.cpp

HEADERS += mainwindow.h \
    addlidardialog.h \
    lidar_driver/driver.h \
    lidar_driver/input.h \
    lidar_driver/rawdata.h \
    lidar_driver/lidar.h

FORMS += mainwindow.ui \
    addlidardialog.ui
