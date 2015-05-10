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
include tools/CMakeFiles/MagneticField.dir/depend.make

# Include the progress variables for this target.
include tools/CMakeFiles/MagneticField.dir/progress.make

# Include the compile flags for this target's objects.
include tools/CMakeFiles/MagneticField.dir/flags.make

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o: tools/CMakeFiles/MagneticField.dir/flags.make
tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o: ../tools/MagneticField.cpp
tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o: man/MagneticField.usage
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/Maverick/Documents/GeographicLib-1.42/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/MagneticField.dir/MagneticField.cpp.o -c /Users/Maverick/Documents/GeographicLib-1.42/tools/MagneticField.cpp

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MagneticField.dir/MagneticField.cpp.i"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/Maverick/Documents/GeographicLib-1.42/tools/MagneticField.cpp > CMakeFiles/MagneticField.dir/MagneticField.cpp.i

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MagneticField.dir/MagneticField.cpp.s"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/Maverick/Documents/GeographicLib-1.42/tools/MagneticField.cpp -o CMakeFiles/MagneticField.dir/MagneticField.cpp.s

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.requires:
.PHONY : tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.requires

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.provides: tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.requires
	$(MAKE) -f tools/CMakeFiles/MagneticField.dir/build.make tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.provides.build
.PHONY : tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.provides

tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.provides.build: tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o

# Object files for target MagneticField
MagneticField_OBJECTS = \
"CMakeFiles/MagneticField.dir/MagneticField.cpp.o"

# External object files for target MagneticField
MagneticField_EXTERNAL_OBJECTS =

tools/MagneticField: tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o
tools/MagneticField: tools/CMakeFiles/MagneticField.dir/build.make
tools/MagneticField: src/libGeographic.14.0.3.dylib
tools/MagneticField: tools/CMakeFiles/MagneticField.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable MagneticField"
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MagneticField.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/CMakeFiles/MagneticField.dir/build: tools/MagneticField
.PHONY : tools/CMakeFiles/MagneticField.dir/build

tools/CMakeFiles/MagneticField.dir/requires: tools/CMakeFiles/MagneticField.dir/MagneticField.cpp.o.requires
.PHONY : tools/CMakeFiles/MagneticField.dir/requires

tools/CMakeFiles/MagneticField.dir/clean:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build/tools && $(CMAKE_COMMAND) -P CMakeFiles/MagneticField.dir/cmake_clean.cmake
.PHONY : tools/CMakeFiles/MagneticField.dir/clean

tools/CMakeFiles/MagneticField.dir/depend:
	cd /Users/Maverick/Documents/GeographicLib-1.42/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Maverick/Documents/GeographicLib-1.42 /Users/Maverick/Documents/GeographicLib-1.42/tools /Users/Maverick/Documents/GeographicLib-1.42/build /Users/Maverick/Documents/GeographicLib-1.42/build/tools /Users/Maverick/Documents/GeographicLib-1.42/build/tools/CMakeFiles/MagneticField.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/CMakeFiles/MagneticField.dir/depend

