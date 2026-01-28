# This line works around a strange bug in GLMConfig-version.cmake which causes
# the GLM_VERSION variable to not be set correctly. Since the file in question
# doesn't quote their variables, an (uncatchable) CMake runtime error results,
# which kills the configuration phase. Setting GLM_VERSION here carries over to
# GLMConfig-version.cmakes (scoping rules), which dodges the issue for now.
set(GLM_VERSION 0.9.9.0)
FIND_PACKAGE(GLM ${GLM_VERSION} QUIET EXACT)

IF(${glm_FOUND})
	INCLUDE_DIRECTORIES(${glm_INCLUDE_DIR})
	message(STATUS "Using System glm")
ELSE(${glm_FOUND})
	IF(EXISTS /usr/include/glm/gtx/extended_min_max.hpp)
		message(STATUS "glm was found in default location")
	ELSE()
		SET(expected_glm_dir ${CMAKE_SOURCE_DIR}/third-party/glm)

		IF(NOT EXISTS ${expected_glm_dir}/copying.txt)
			EXECUTE_PROCESS(COMMAND git clone -b 0.9.9.7 --single-branch --depth 1 https://github.com/g-truc/glm.git ${expected_glm_dir})
		ENDIF()

		IF(NOT EXISTS ${expected_glm_dir}/glm/gtx/extended_min_max.hpp)
			EXECUTE_PROCESS(COMMAND rm -rf ${expected_glm_dir})
			EXECUTE_PROCESS(COMMAND git clone -b 0.9.9.7 --single-branch --depth 1 https://github.com/g-truc/glm.git ${expected_glm_dir})
		ENDIF()

		INCLUDE_DIRECTORIES(BEFORE SYSTEM ${expected_glm_dir})
		message(STATUS "Using bundled glm at ${expected_glm_dir}")
	ENDIF()
ENDIF(${glm_FOUND})

add_definitions(-DGLM_ENABLE_EXPERIMENTAL -DGLM_FORCE_SIZE_FUNC=1 -DGLM_FORCE_RADIANS=1)

# vim: tw=78
