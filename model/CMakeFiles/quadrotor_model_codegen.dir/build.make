# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model

# Include any dependencies generated for this target.
include CMakeFiles/quadrotor_model_codegen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/quadrotor_model_codegen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/quadrotor_model_codegen.dir/flags.make

CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o: CMakeFiles/quadrotor_model_codegen.dir/flags.make
CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o: quadrotor_model_thrustrates.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o -c /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/quadrotor_model_thrustrates.cpp

CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/quadrotor_model_thrustrates.cpp > CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.i

CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/quadrotor_model_thrustrates.cpp -o CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.s

# Object files for target quadrotor_model_codegen
quadrotor_model_codegen_OBJECTS = \
"CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o"

# External object files for target quadrotor_model_codegen
quadrotor_model_codegen_EXTERNAL_OBJECTS =

quadrotor_model_codegen: CMakeFiles/quadrotor_model_codegen.dir/quadrotor_model_thrustrates.cpp.o
quadrotor_model_codegen: CMakeFiles/quadrotor_model_codegen.dir/build.make
quadrotor_model_codegen: /home/alvaro/workspaces/ACADOtoolkit/build/lib/libacado_toolkit_s.so
quadrotor_model_codegen: /home/alvaro/workspaces/ACADOtoolkit/build/lib/libacado_casadi.a
quadrotor_model_codegen: /home/alvaro/workspaces/ACADOtoolkit/build/lib/libacado_csparse.a
quadrotor_model_codegen: CMakeFiles/quadrotor_model_codegen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable quadrotor_model_codegen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/quadrotor_model_codegen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/quadrotor_model_codegen.dir/build: quadrotor_model_codegen

.PHONY : CMakeFiles/quadrotor_model_codegen.dir/build

CMakeFiles/quadrotor_model_codegen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/quadrotor_model_codegen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/quadrotor_model_codegen.dir/clean

CMakeFiles/quadrotor_model_codegen.dir/depend:
	cd /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model /home/alvaro/workspaces/rpg_mpc_ws/src/rpg_mpc/model/CMakeFiles/quadrotor_model_codegen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/quadrotor_model_codegen.dir/depend

