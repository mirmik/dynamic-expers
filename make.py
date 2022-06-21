#!/usr/bin/env python3

import licant

#licant.include("rabbit")
#licant.include("ralgo")

licant.cxx_application("app",
	sources = ["src/main.cpp"],
	libs = ["reactphysics3d", "GL", "GLEW", "glfw", "igris", "nos", "rabbit", "ralgo"],
#	mdepends = ["rabbit", "rabbit.opengl", "ralgo"] 
)

licant.ex("app")