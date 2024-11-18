PKGDIR		?= ../..
L4DIR		?= $(PKGDIR)/../..

TARGET		= $(PKGNAME)

# list your .c or .cc files here
SRC_C		=
SRC_CC          = main.cc

# list requirements of your program here
REQUIRES_LIBS   = lib_i2c_server libstdc++ libpthread libio-vbus

PRIVATE_INCDIR += $(PKGDIR)/lib/src

include $(L4DIR)/mk/prog.mk
