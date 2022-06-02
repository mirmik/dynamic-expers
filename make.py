#!/usr/bin/env python3

import licant

licant.include("rabbit")

licant.cxx_application("app",
	sources = ["src/main.cpp"],
	libs = ["reactphysics3d", "GL", "GLEW", "glfw", "igris", "nos", "ralgo"],
	mdepends = ["rabbit", "rabbit.opengl"] 
)

licant.ex("app")