LIBS = -lGLEW -lglfw
FWKS = -framework Cocoa -framework OpenGL -framework IOKit 

Mass-Spring: Source/glSetup.cpp Source/Mass-Spring.cpp Source/glSetup.h
	g++ -o EXE/$@ Source/$@.cpp Source/glSetup.cpp $(LIBS) $(FWKS)
