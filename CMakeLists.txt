cmake_minimum_required(VERSION 3.16)
project(modelconverter VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(LIB_SRCS
	src/mc_common.c
	src/mc_image.c

	src/mc_md3.c
	src/mc_iqm.c
	src/mc_mdr.c

	src/mc_gltf_read.c
	src/mc_gltf_write.c

	src/mc_q3shader.c
	src/mc_animcfg.c
	src/mc_player.c

	src/mc_subdivide.c
	src/mc_decimate.c
)

add_library(modelconverter_lib STATIC ${LIB_SRCS})
target_include_directories(modelconverter_lib PUBLIC src)

find_library(MATH_LIBRARY m)
if (MATH_LIBRARY)
	target_link_libraries(modelconverter_lib PUBLIC ${MATH_LIBRARY})
endif()

add_executable(${PROJECT_NAME} src/modelconverter.c)
target_link_libraries(${PROJECT_NAME} PRIVATE modelconverter_lib)

# ---- Tests ----
enable_testing()

add_executable(test_roundtrip tests/test_roundtrip.c)
target_link_libraries(test_roundtrip PRIVATE modelconverter_lib)
add_test(NAME roundtrip
	COMMAND test_roundtrip ${CMAKE_BINARY_DIR}
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

add_test(NAME cli_help
	COMMAND ${PROJECT_NAME} --help)

# Validate the cube produced by the round-trip test (depends on it
# having run first, so we re-emit it here as a separate target).
add_test(NAME validate_cube
	COMMAND ${PROJECT_NAME} --validate -i ${CMAKE_BINARY_DIR}/cube.md3)
set_tests_properties(validate_cube PROPERTIES DEPENDS roundtrip)

add_test(NAME validate_cube_subcmd
	COMMAND ${PROJECT_NAME} validate ${CMAKE_BINARY_DIR}/cube.md3)
set_tests_properties(validate_cube_subcmd PROPERTIES DEPENDS roundtrip)

add_test(NAME info_cube_subcmd
	COMMAND ${PROJECT_NAME} info ${CMAKE_BINARY_DIR}/cube.md3)
set_tests_properties(info_cube_subcmd PROPERTIES DEPENDS roundtrip)

add_test(NAME convert_cube_md3_to_glb
	COMMAND ${PROJECT_NAME} convert ${CMAKE_BINARY_DIR}/cube.md3 ${CMAKE_BINARY_DIR}/cube_subcmd.glb)
set_tests_properties(convert_cube_md3_to_glb PROPERTIES DEPENDS roundtrip)

