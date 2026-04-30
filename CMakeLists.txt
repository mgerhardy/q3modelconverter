cmake_minimum_required(VERSION 3.16)
project(modelconverter VERSION 1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- Compiler warnings ----
if (MSVC)
	add_compile_options(/W4 /WX /wd4100 /wd4244 /wd4267 /wd4706)
	add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
	add_compile_options(
		-Wall -Wextra -Werror
		-Wshadow
		-Wformat=2
		-Wstrict-prototypes
		-Wmissing-prototypes
		-Wno-unused-parameter
		-Wno-format-truncation
	)
	if (CMAKE_C_COMPILER_ID MATCHES "Clang")
		add_compile_options(-Wno-gnu-zero-variadic-macro-arguments)
	endif()
endif()

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
	src/mc_pk3.c

	src/mc_subdivide.c
	src/mc_decimate.c

	src/miniz/miniz.c
	src/miniz/miniz_tdef.c
	src/miniz/miniz_tinfl.c
	src/miniz/miniz_zip.c
)

add_library(modelconverter_lib STATIC ${LIB_SRCS})
target_include_directories(modelconverter_lib PUBLIC src)
# Suppress warnings in vendored miniz sources.
if (MSVC)
	set_source_files_properties(
		src/miniz/miniz.c src/miniz/miniz_tdef.c
		src/miniz/miniz_tinfl.c src/miniz/miniz_zip.c
		PROPERTIES COMPILE_FLAGS "/w"
	)
else()
	set_source_files_properties(
		src/miniz/miniz.c src/miniz/miniz_tdef.c
		src/miniz/miniz_tinfl.c src/miniz/miniz_zip.c
		PROPERTIES COMPILE_FLAGS "-w"
	)
endif()

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

# Full player-bundle round-trip (only when worldofpadman/ subdir is present).
if (EXISTS "${CMAKE_SOURCE_DIR}/worldofpadman/wop/models.pk3dir/models/wop_players/padman/lower.md3")
	add_test(NAME roundtrip_player
		COMMAND ${CMAKE_SOURCE_DIR}/scripts/roundtrip-test.sh
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
	set_tests_properties(roundtrip_player PROPERTIES DEPENDS roundtrip)
endif()


# ---- Install ----
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
install(TARGETS modelconverter_lib ARCHIVE DESTINATION lib)
install(FILES src/modelconverter.h DESTINATION include/modelconverter)
install(FILES docs/modelconverter.1 DESTINATION share/man/man1)
install(FILES contrib/completions/modelconverter.bash
        DESTINATION share/bash-completion/completions RENAME modelconverter)
install(FILES contrib/completions/_modelconverter
        DESTINATION share/zsh/site-functions)

# ---- CPack (deb/rpm) ----
set(CPACK_PACKAGE_NAME "modelconverter")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Quake 3 model format converter (MD3/IQM/MDR/glTF)")
set(CPACK_PACKAGE_CONTACT "martin.gerhardy@gmail.com")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/mgerhardy/q3modelconverter")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE.md")

set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

set(CPACK_RPM_PACKAGE_LICENSE "GPL-2.0-or-later")
set(CPACK_RPM_PACKAGE_GROUP "Applications/Multimedia")
set(CPACK_RPM_FILE_NAME RPM-DEFAULT)

include(CPack)
