VESC_TOOL ?= vesc_tool

TARGET = balance

SOURCES = main.c 
SOURCES += conf/buffer.c 
SOURCES += conf/confparser.c 
SOURCES += conf/confxml.c 
SOURCES += util/biquad.c 
SOURCES += util/realtimedata.c 
SOURCES += modifiers/torquetilt.c 
SOURCES += modifiers/turntilt.c 
SOURCES += modifiers/speedtilt.c 
SOURCES += modifiers/basictilt.c 
SOURCES += modifiers/untilt.c

ADD_TO_CLEAN = conf/confxml.h conf/confxml.c

VESC_C_LIB_PATH=../../c_libs/
include $(VESC_C_LIB_PATH)rules.mk

main.c: conf/confparser.h conf/confxml.h

conf/confparser.h conf/confxml.h conf/confxml.c &: conf/settings.xml
	$(VESC_TOOL) --xmlConfToCode conf/settings.xml

