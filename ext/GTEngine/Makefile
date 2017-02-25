CFG ?= Debug

CC := /usr/bin/g++-5
CFLAGS += -c -std=c++14 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH
INCLUDE := -I Include
LIBDIR := lib/$(CFG)

ifeq (Debug,$(findstring Debug,$(CFG)))
CFLAGS += -g -D_DEBUG
else
CFLAGS += -O2 -DNDEBUG
endif

ifeq (Dynamic,$(findstring Dynamic,$(CFG)))
SOVER = 3
RELVER = $(SOVER).6
CFLAGS += -fPIC
AR := $(CC)
ARFLAGS := $(LDFLAGS) -shared -fPIC -Wl,-soname,libgtengine.so.$(SOVER) -o
LIB := $(LIBDIR)/libgtengine.so.$(RELVER)
else
AR := /usr/bin/ar
ARFLAGS := rs
LIB := $(LIBDIR)/libgtengine.a
endif

OBJDIR := obj/$(CFG)

SRC_ROOT := Source
SRC_ROOT_FILES := $(wildcard $(SRC_ROOT)/*.cpp)
OBJ_ROOT_FILES := $(notdir $(SRC_ROOT_FILES))
OBJ_ROOT_FILES := $(OBJ_ROOT_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_LOWLEVEL := Source/LowLevel
SRC_LOWLEVEL_FILES := $(wildcard $(SRC_LOWLEVEL)/*.cpp)
OBJ_LOWLEVEL_FILES := $(notdir $(SRC_LOWLEVEL_FILES))
OBJ_LOWLEVEL_FILES := $(OBJ_LOWLEVEL_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_MATHEMATICS := Source/Mathematics
SRC_MATHEMATICS_FILES := $(wildcard $(SRC_MATHEMATICS)/*.cpp)
OBJ_MATHEMATICS_FILES := $(notdir $(SRC_MATHEMATICS_FILES))
OBJ_MATHEMATICS_FILES := $(OBJ_MATHEMATICS_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_IMAGICS := Source/Imagics
SRC_IMAGICS_FILES := $(wildcard $(SRC_IMAGICS)/*.cpp)
OBJ_IMAGICS_FILES := $(notdir $(SRC_IMAGICS_FILES))
OBJ_IMAGICS_FILES := $(OBJ_IMAGICS_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_PHYSICS := Source/Physics
SRC_PHYSICS_FILES := $(wildcard $(SRC_PHYSICS)/*.cpp)
OBJ_PHYSICS_FILES := $(notdir $(SRC_PHYSICS_FILES))
OBJ_PHYSICS_FILES := $(OBJ_PHYSICS_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_GRAPHICS := Source/Graphics
SRC_GRAPHICS_FILES := $(wildcard $(SRC_GRAPHICS)/*.cpp)
OBJ_GRAPHICS_FILES := $(notdir $(SRC_GRAPHICS_FILES))
OBJ_GRAPHICS_FILES := $(OBJ_GRAPHICS_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_GRAPHICS_OPENGL := Source/Graphics/GL4
SRC_GRAPHICS_OPENGL_FILES := $(wildcard $(SRC_GRAPHICS_OPENGL)/*.cpp)
OBJ_GRAPHICS_OPENGL_FILES := $(notdir $(SRC_GRAPHICS_OPENGL_FILES))
OBJ_GRAPHICS_OPENGL_FILES := $(OBJ_GRAPHICS_OPENGL_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_GRAPHICS_GLX := Source/Graphics/GL4/GLX
SRC_GRAPHICS_GLX_FILES := $(wildcard $(SRC_GRAPHICS_GLX)/*.cpp)
OBJ_GRAPHICS_GLX_FILES := $(notdir $(SRC_GRAPHICS_GLX_FILES))
OBJ_GRAPHICS_GLX_FILES := $(OBJ_GRAPHICS_GLX_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_APPLICATIONS := Source/Applications
SRC_APPLICATIONS_FILES := $(wildcard $(SRC_APPLICATIONS)/*.cpp)
OBJ_APPLICATIONS_FILES := $(notdir $(SRC_APPLICATIONS_FILES))
OBJ_APPLICATIONS_FILES := $(OBJ_APPLICATIONS_FILES:%.cpp=$(OBJDIR)/%.o)

SRC_APPLICATIONS_GLX := Source/Applications/GLX
SRC_APPLICATIONS_GLX_FILES := $(wildcard $(SRC_APPLICATIONS_GLX)/*.cpp)
OBJ_APPLICATIONS_GLX_FILES := $(notdir $(SRC_APPLICATIONS_GLX_FILES))
OBJ_APPLICATIONS_GLX_FILES := $(OBJ_APPLICATIONS_GLX_FILES:%.cpp=$(OBJDIR)/%.o)

OBJ := \
$(OBJ_ROOT_FILES) \
$(OBJ_LOWLEVEL_FILES) \
$(OBJ_MATHEMATICS_FILES) \
$(OBJ_IMAGICS_FILES) \
$(OBJ_PHYSICS_FILES) \
$(OBJ_GRAPHICS_FILES) \
$(OBJ_GRAPHICS_OPENGL_FILES) \
$(OBJ_GRAPHICS_GLX_FILES) \
$(OBJ_APPLICATIONS_FILES) \
$(OBJ_APPLICATIONS_GLX_FILES)

build : createdir compile setlinks

createdir :
	@mkdir -p $(LIBDIR)
	@mkdir -p $(OBJDIR)

compile : $(OBJ)
	$(AR) $(ARFLAGS) $(LIB) $(OBJDIR)/*.o

setlinks :
	# The shared libraries are created with extension .so.$(RELVER).$(SOVER), so
	# we need soft links for the applications to find them at run time.
	cd $(LIBDIR) && \
	(FILES=`find . -name "*.so.$(RELVER)"`;\
	ls $$FILES;\
	for file in $$FILES; do\
		ln -sfv $$file `basename $$file .so.$(RELVER)`.so.$(SOVER);\
		ln -sfv $$file `basename $$file .so.$(RELVER)`.so;\
		done)
	cd ../..

clean :
	rm -rf $(LIBDIR)
	rm -rf $(OBJDIR)

$(OBJDIR)/%.o : $(SRC_ROOT)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_LOWLEVEL)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_MATHEMATICS)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_IMAGICS)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_PHYSICS)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_GRAPHICS)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_GRAPHICS_OPENGL)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_GRAPHICS_GLX)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_APPLICATIONS)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : $(SRC_APPLICATIONS_GLX)/%.cpp
	$(CC) $(INCLUDE) $(CFLAGS) $< -o $@
