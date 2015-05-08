# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Maverick/Documents/GeographicLib-1.42

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Maverick/Documents/GeographicLib-1.42/build

# Include any dependencies generated for this target.
include tools/CMakeFiles/RhumbSolve.dir/depend.make

# Include the progress variables for this target.
include tools/CMakeFiles/RhumbSolve.dir/progress.make

# Include the compile flags for this target's objects.
include tools/CMakeFiles/RhumbSolve.dir/flags.make

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o: tools/CMakeFiles/RhumbSolve.dir/flags.make
tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o: ../tools/RhumbSolve.cpp
tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o: man/RhumbSolve.usage
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/Maverick/Documents/GeographicLib-1.42/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o -c /Users/Maverick/Documents/GeographicLib-1.42/tools/RhumbSolve.cpp

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.i"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/Maverick/Documents/GeographicLib-1.42/tools/RhumbSolve.cpp > CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.i

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.s"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/Maverick/Documents/GeographicLib-1.42/tools/RhumbSolve.cpp -o CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.s

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.requires:
.PHONY : tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.requires

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.provides: tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.requires
	$(MAKE) -f tools/CMakeFiles/RhumbSolve.dir/build.make tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.provides.build
.PHONY : tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.provides

tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.provides.build: tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o

# Object files for target RhumbSolve
RhumbSolve_OBJECTS = \
"CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o"

# External object files for target RhumbSolve
RhumbSolve_EXTERNAL_OBJECTS =

tools/RhumbSolve: tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o
tools/RhumbSolve: tools/CMakeFiles/RhumbSolve.dir/build.make
tools/RhumbSolve: src/libGeographic.14.0.3.dylib
tools/RhumbSolve: tools/CMakeFiles/RhumbSolve.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable RhumbSolve"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RhumbSolve.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/CMakeFiles/RhumbSolve.dir/build: tools/RhumbSolve
.PHONY : tools/CMakeFiles/RhumbSolve.dir/build

tools/CMakeFiles/RhumbSolve.dir/requires: tools/CMakeFiles/RhumbSolve.dir/RhumbSolve.cpp.o.requires
.PHONY : tools/CMakeFiles/RhumbSolve.dir/requires

tools/CMakeFiles/RhumbSolve.dir/clean:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && $(CMAKE_COMMAND) -P CMakeFiles/RhumbSolve.dir/cmake_clean.cmake
.PHONY : tools/CMakeFiles/RhumbSolve.dir/clean

tools/CMakeFiles/RhumbSolve.dir/depend:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Maverick/Documents/GeographicLib-1.42 /Users/Maverick/Documents/GeographicLib-1.42/tools /Users/Maverick/Documents/GeographicLib-1.42/build /Users/Maverick/Documents/GeographicLib-1.42/build/tools /Users/Maverick/Documents/GeographicLib-1.42/build/tools/CMakeFiles/RhumbSolve.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/CMakeFiles/RhumbSolve.dir/depend

