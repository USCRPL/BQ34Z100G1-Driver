set(BQ34Z100_SOURCES
	BQ34Z100.cpp
	BQ34Z100.h)

add_library(BQ34Z100 ${BQ34Z100_SOURCES})
target_include_directories(BQ34Z100 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(BQ34Z100 mbed-core-flags)
