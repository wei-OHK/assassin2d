VERSION_MAJOR = 1
VERSION_MINOR = 0
VERSION_RELEASE = 0
# assassin2d 1.0.0
# Makefile for Linux
#
# To use, just run:
#
# make
#
# This will compile assassin2d, create a static lib (libassassin2d.a), and copy it
# into the subdirectory lib/Linux. That's all.
#
# If you want assassin2d to be compiled as shared lib (libassassin2d.so.versionnumber), then run:
#
# make sharedlib
# make install
#
# If you want to compile in debug mode run:
#
# make DEBUG=1
#

#List of subdirs
VPATH = ../source/assassin2d

#List of object files
LINKOBJ = actor.o

###############
#Compiler flags
CXXFLAGS += -Wall -std=c++11 -MMD -fPIC
ifndef DEBUG
CXXFLAGS += -fexpensive-optimizations -O3
else
CXXFLAGS += -g -D_DEBUG
endif
CFLAGS := -O3 -fexpensive-optimizations -fPIC

#Linux specific options
staticlib sharedlib install: SYSTEM = Linux
STATIC_LIB = libassassin2d.a
LIB_PATH = ../lib/$(SYSTEM)
INSTALL_DIR = /usr/lib
SHARED_LIB = libassassin2d.so

VERSION = $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_RELEASE)
SHARED_FULLNAME = $(SHARED_LIB).$(VERSION)
SONAME = $(SHARED_LIB).$(VERSION_MAJOR).$(VERSION_MINOR)

####################
# All target, builds assassin2d as static lib (libassassin2d.a) and copies it into lib/Linux
all: staticlib

# Builds assassin2d as shared lib (libassassin2d.so.versionNumber) and copies it into lib/Linux
sharedlib: $(LINKOBJ)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -shared -Wl,-soname,$(SONAME) -o $(SHARED_FULLNAME) $^
	mkdir -p $(LIB_PATH)
	cp $(SHARED_FULLNAME) $(LIB_PATH)

# Builds assassin2d as static lib (libassassin2d.a)
$(STATIC_LIB): $(LINKOBJ)
	$(AR) rs $@ $^

# Copies static lib into lib/Linux
staticlib: $(STATIC_LIB)
	mkdir -p $(LIB_PATH)
	cp $^ $(LIB_PATH)

# Installs assassin2d if it was created as shared lib
install:
	$(RM) -r $(INSTALL_DIR)/../include/assassin2d
	cp -r ../include/assassin2d $(INSTALL_DIR)/../include/
	cp $(LIB_PATH)/$(SHARED_FULLNAME) $(INSTALL_DIR)
	cd $(INSTALL_DIR) && ln -s -f $(SHARED_FULLNAME) $(SONAME)
	cd $(INSTALL_DIR) && ln -s -f $(SONAME) $(SHARED_LIB)
#	ldconfig -n $(INSTALL_DIR)

help:
	@echo "Available targets for assassin2d"
	@echo " sharedlib: Build shared library libassassin2d.so for Linux"
	@echo " staticlib: Build static library libassassin2d.a for Linux"
	@echo " install: Copy shared library to /usr/lib"
	@echo ""
	@echo " clean: Clean up directory"

# Cleans all temporary files and compilation results.
clean:
	$(RM) $(LINKOBJ) $(SHARED_FULLNAME) $(STATIC_LIB) $(LINKOBJ:.o=.d)

.PHONY: all sharedlib staticlib help install clean

