#-------------------------------------------------
#
# Project created by QtCreator 2013-04-05T19:35:15
#
#-------------------------------------------------

QT       += core gui serialport widgets

TARGET = GUIPanel
TEMPLATE = app


SOURCES += main.cpp\
        guipanel.cpp \
    crc.c \
    protocol.c \
    qremotetiva.cpp

HEADERS  += guipanel.h \
    crc.h \
    protocol.h \
    qremotetiva.h

FORMS    += guipanel.ui

CONFIG   +=qwt # Añadir para usar Qwidgets

win32:debug {
    LIBS += -lqwtd -lanalogwidgetsd -lColorWidgets-qt51d# Añadido Necesario para encontrar implementación de librerias (si estoy en windows y en modo debug)!!
}

win32:release {
    LIBS += -lqwt -lanalogwidgets -lColorWidgets-qt51# Añadido Necesario para encontrar implementación de librerias (si estoy en windows y en modo release)!!
}

unix:!macx {
    LIBS += -lqwt -lanalogwidgets -lColorWidgets-qt5 # Añadido Necesario para encontrar implementación de librerias (Linux)!!
}

#Nota MacOS: CAMBIA EL PATH para que coincida con el usuario!!!
macx: {
LIBS += -framework qwt -lColorWidgets-qt5.1 -lanalogwidgets -L/Users/angel/Qt/5.6/clang_64/lib/
INCLUDEPATH += /Users/angel/Qt/5.6/clang_64/include/
INCLUDEPATH += /Users/angel/Qt/5.6/clang_64/lib/qwt.framework/Headers
}
