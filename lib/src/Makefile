PKGDIR?= ../..
L4DIR ?= $(PKGDIR)/../..

# the name of your library
TARGET	= lib_i2c_server.a

# list your .c or .cc files here
SRC_C	=
SRC_CC  = i2c_server.cc

PC_FILENAME = lib_i2c_server
REQUIRES_LIBS = libstdc++ l4virtio

include $(L4DIR)/mk/lib.mk
