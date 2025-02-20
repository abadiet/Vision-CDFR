cmake_minimum_required(VERSION 3.5)

# Project
project(Vision)

# Options
option(CUDA "Enable CUDA" OFF)
if (CUDA)
	message(STATUS "CUDA enabled")
endif()

# Sources
file(GLOB_RECURSE SOURCES src/*.cpp src/*.c)
file(GLOB_RECURSE HEADERS include/*.hpp include/*.h)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Flags
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
target_compile_options(${PROJECT_NAME} PRIVATE
	$<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
	$<$<CXX_COMPILER_ID:Clang>:-O3 -Wall -Wextra -Werror -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wconversion -Wunreachable-code -Wpedantic -Wshadow -Wwrite-strings -Wcast-qual -Wstrict-aliasing -Wpointer-arith -Wformat=2 -Weverything -Wextra-semi -Wmissing-prototypes -Wunreachable-code-aggressive -Wimplicit-fallthrough -Wloop-analysis -Wrange-loop-analysis -Wcomma -Wmove -Wconditional-uninitialized -Wassign-enum -Wunused-exception-parameter -Wcovered-switch-default -Wno-error=c++98-compat -Wno-error=global-constructors -Wno-error=exit-time-destructors -Wno-error=padded>
	$<$<CXX_COMPILER_ID:Gnu>: -O3 -Wall -Wextra -Werror -Wfloat-equal -Wundef -Wcast-align -Wwrite-strings -Wconversion -Wunreachable-code -Wpedantic -Wshadow -Wwrite-strings -Wcast-qual -Wstrict-aliasing -Wpointer-arith -Wformat=2 -Wextra-semi -Wmissing-prototypes -Wimplicit-fallthrough -Wno-error=c++98-compat -Wno-error=global-constructors -Wno-error=exit-time-destructors -Wno-error=padded>
)
if (CUDA)
	target_compile_definitions(${PROJECT_NAME} PRIVATE CUDA)
endif()

# Add Libraries
find_package(OpenCV REQUIRED)

# Includes
target_include_directories(${PROJECT_NAME} PUBLIC include ${OpenCV_INCLUDE_DIRS})

# Link Libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
