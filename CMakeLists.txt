cmake_minimum_required (VERSION 3.10.2)

##### PLUGIN NAME #####
set(pluginname gipBulletPhysics)


##### PLUGIN PATHS #####
set(PLUGIN_DIR ${PLUGINS_DIR}/${pluginname})
set(PLUGIN_LIBS_DIR ${PLUGIN_DIR}/libs)
set(PLUGIN_PREBUILTS_DIR ${PLUGIN_DIR}/prebuilts)


##### PLUGIN SOURCES #####
list(APPEND PLUGIN_SRCS
			${PLUGIN_DIR}/src/gipBulletPhysics.cpp
			${PLUGIN_DIR}/src/gipDebugDraw.cpp
			${PLUGIN_DIR}/src/gipBaseGameObject.cpp
			${PLUGIN_DIR}/src/gImageGameObject.cpp
			${PLUGIN_DIR}/src/gModelGameObject.cpp
			${PLUGIN_DIR}/src/gGhostImageGameObject.cpp
			${PLUGIN_DIR}/src/gGhostModelGameObject.cpp
)


##### PLUGIN HEADERS #####
list(APPEND PLUGIN_INCLUDES
			${PLUGIN_DIR}/src
)


##### BULLET FROM SOURCE #####
# iOS, the iOS simulator and Android have no prebuilt Bullet binaries shipped with
# this plugin, so Bullet is checked out and compiled together with the engine there.
# Any other platform can opt in with -DGIP_BULLET_FROM_SOURCE=ON.
if(IOS OR ANDROID OR LINUX)
	set(GIP_BULLET_FROM_SOURCE_DEFAULT ON)
else()
	set(GIP_BULLET_FROM_SOURCE_DEFAULT OFF)
endif()

option(GIP_BULLET_FROM_SOURCE "Compile Bullet from source instead of using the prebuilt binaries" ${GIP_BULLET_FROM_SOURCE_DEFAULT})
set(GIP_BULLET_GIT_REPOSITORY "https://github.com/bulletphysics/bullet3.git" CACHE STRING "Bullet repository to check out")
set(GIP_BULLET_GIT_TAG "3.25" CACHE STRING "Bullet tag/branch to check out")
set(GIP_BULLET_SOURCE_DIR "${PLUGIN_DIR}/deps/bullet3" CACHE PATH "Where the Bullet sources live. Put a submodule here to skip the download.")

if(GIP_BULLET_FROM_SOURCE)
	if(CMAKE_VERSION VERSION_LESS 3.14)
		message(FATAL_ERROR "gipBulletPhysics: building Bullet from source needs CMake 3.14 or newer.")
	endif()

	# Fetch the sources unless they are already there. A git submodule placed in
	# GIP_BULLET_SOURCE_DIR is picked up as is, nothing is downloaded in that case.
	# The stamp file marks a checkout we made ourselves, so bumping GIP_BULLET_GIT_TAG
	# refreshes it while a submodule is never touched.
	set(GIP_BULLET_STAMP_FILE "${GIP_BULLET_SOURCE_DIR}.tag")
	set(GIP_BULLET_NEEDS_FETCH OFF)
	if(NOT EXISTS "${GIP_BULLET_SOURCE_DIR}/CMakeLists.txt")
		set(GIP_BULLET_NEEDS_FETCH ON)
	elseif(EXISTS "${GIP_BULLET_STAMP_FILE}")
		file(READ "${GIP_BULLET_STAMP_FILE}" GIP_BULLET_STAMPED_TAG)
		string(STRIP "${GIP_BULLET_STAMPED_TAG}" GIP_BULLET_STAMPED_TAG)
		if(NOT GIP_BULLET_STAMPED_TAG STREQUAL GIP_BULLET_GIT_TAG)
			set(GIP_BULLET_NEEDS_FETCH ON)
		endif()
	endif()

	if(GIP_BULLET_NEEDS_FETCH)
		find_package(Git REQUIRED)
		message(STATUS "gipBulletPhysics: fetching Bullet ${GIP_BULLET_GIT_TAG} into ${GIP_BULLET_SOURCE_DIR}")
		file(REMOVE_RECURSE "${GIP_BULLET_SOURCE_DIR}")

		# Blobless + sparse checkout, this pulls ~15 MB instead of the ~450 MB the full
		# tree weighs. Android configures once per ABI, so the difference is noticeable.
		execute_process(
				COMMAND ${GIT_EXECUTABLE} clone --depth 1 --branch ${GIP_BULLET_GIT_TAG}
						--filter=blob:none --sparse ${GIP_BULLET_GIT_REPOSITORY} ${GIP_BULLET_SOURCE_DIR}
				RESULT_VARIABLE GIP_BULLET_CLONE_RESULT
		)
		if(GIP_BULLET_CLONE_RESULT EQUAL 0)
			execute_process(
					COMMAND ${GIT_EXECUTABLE} sparse-checkout set src
					WORKING_DIRECTORY ${GIP_BULLET_SOURCE_DIR}
					RESULT_VARIABLE GIP_BULLET_CLONE_RESULT
			)
		endif()

		# Older git or a server without partial clone support, fall back to a plain
		# shallow clone.
		if(NOT GIP_BULLET_CLONE_RESULT EQUAL 0)
			file(REMOVE_RECURSE "${GIP_BULLET_SOURCE_DIR}")
			execute_process(
					COMMAND ${GIT_EXECUTABLE} clone --depth 1 --branch ${GIP_BULLET_GIT_TAG}
							${GIP_BULLET_GIT_REPOSITORY} ${GIP_BULLET_SOURCE_DIR}
					RESULT_VARIABLE GIP_BULLET_CLONE_RESULT
			)
		endif()

		if(NOT GIP_BULLET_CLONE_RESULT EQUAL 0)
			message(FATAL_ERROR "gipBulletPhysics: could not check out Bullet from ${GIP_BULLET_GIT_REPOSITORY}")
		endif()
		file(WRITE "${GIP_BULLET_STAMP_FILE}" "${GIP_BULLET_GIT_TAG}")
	endif()

	# Bullet builds demos, extras, tests and the Bullet 3 libraries by default, none of
	# which link on a phone. These have to go into the cache because Bullet asks for
	# CMake 2.4 compatibility, which makes its option() calls ignore normal variables.
	foreach(GIP_BULLET_OFF_OPTION
			BUILD_BULLET2_DEMOS BUILD_CPU_DEMOS BUILD_OPENGL3_DEMOS BUILD_EXTRAS
			BUILD_UNIT_TESTS BUILD_BULLET3 BUILD_PYBULLET BUILD_ENET BUILD_CLSOCKET
			BUILD_EGL BUILD_SHARED_LIBS INSTALL_LIBS INSTALL_CMAKE_FILES
			USE_GRAPHICAL_BENCHMARK USE_GLUT USE_DOUBLE_PRECISION)
		set(${GIP_BULLET_OFF_OPTION} OFF CACHE BOOL "" FORCE)
	endforeach()

	# Bullet 3.25 declares cmake_minimum_required(VERSION 2.4.3), which CMake 4 refuses
	# to configure without this. CMake older than 3.31 ignores the variable and accepts
	# the old declaration anyway.
	set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
	add_subdirectory(${GIP_BULLET_SOURCE_DIR} ${CMAKE_BINARY_DIR}/bullet3 EXCLUDE_FROM_ALL)
	unset(CMAKE_POLICY_VERSION_MINIMUM)

	# The engine is a shared library, so the Bullet static libs going into it have to be
	# position independent. Without this, Bullet's thread local storage resolves to the
	# local-exec model and the link fails with an R_X86_64_TPOFF32 relocation error.
	foreach(GIP_BULLET_TARGET BulletDynamics BulletCollision LinearMath)
		if(TARGET ${GIP_BULLET_TARGET})
			set_target_properties(${GIP_BULLET_TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
		endif()
	endforeach()

	# Our headers include Bullet as "bullet/btBulletDynamicsCommon.h" while upstream
	# keeps those headers directly under src/, so hand out a directory that has src/
	# linked in as "bullet".
	set(GIP_BULLET_INCLUDE_ROOT ${CMAKE_BINARY_DIR}/gipBulletPhysics-include)
	if(NOT EXISTS ${GIP_BULLET_INCLUDE_ROOT}/bullet/btBulletDynamicsCommon.h)
		file(MAKE_DIRECTORY ${GIP_BULLET_INCLUDE_ROOT})
		file(CREATE_LINK ${GIP_BULLET_SOURCE_DIR}/src ${GIP_BULLET_INCLUDE_ROOT}/bullet
				SYMBOLIC RESULT GIP_BULLET_LINK_RESULT)
		if(NOT GIP_BULLET_LINK_RESULT EQUAL 0)
			# No symlink support on this filesystem, copy the headers over instead.
			file(COPY ${GIP_BULLET_SOURCE_DIR}/src/ DESTINATION ${GIP_BULLET_INCLUDE_ROOT}/bullet
					FILES_MATCHING PATTERN "*.h")
		endif()
	endif()

	list(APPEND PLUGIN_INCLUDES
				${GIP_BULLET_INCLUDE_ROOT}
				${GIP_BULLET_SOURCE_DIR}/src
	)
elseif(WIN32)
	list(APPEND PLUGIN_INCLUDES
			${PLUGIN_DIR}/src
			${PLUGIN_LIBS_DIR}/include
			${PLUGIN_LIBS_DIR}/include/bullet
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet2FileLoader
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet2FileLoader/autogenerated
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Collision/BroadPhaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Collision/BroadPhaseCollision/shared
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Collision/NarrowPhaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Collision/NarrowPhaseCollision/shared
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Common
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Common/shared
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Dynamics
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Dynamics/ConstraintSolver
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Dynamics/shared
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3Geometry
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/BroadPhaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/BroadPhaseCollision/kernels
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/Initialize
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/NarrowphaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/NarrowphaseCollision/kernels
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/ParallelPrimitives
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/ParallelPrimitives/kernels
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/Raycast
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/Raycast/kernels
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/Rigidbody
			${PLUGIN_LIBS_DIR}/include/bullet/Bullet3OpenCL/Rigidbody/kernels
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision/BroadPhaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision/CollisionDispatch
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision/CollisionShapes
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision/Gimpact
			${PLUGIN_LIBS_DIR}/include/bullet/BulletCollision/NarrowPhaseCollision
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/Character
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/ConstraintSolver
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/Dynamics
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/Featherstone
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/MLCPSolvers
			${PLUGIN_LIBS_DIR}/include/bullet/BulletDynamics/Vehicle
			${PLUGIN_LIBS_DIR}/include/bullet/BulletFileLoader
			${PLUGIN_LIBS_DIR}/include/bullet/BulletFileLoader/autogenerated
			${PLUGIN_LIBS_DIR}/include/bullet/BulletInverseDynamics
			${PLUGIN_LIBS_DIR}/include/bullet/BulletSoftBody
			${PLUGIN_LIBS_DIR}/include/bullet/BulletWorldImporter
			${PLUGIN_LIBS_DIR}/include/bullet/BulletXmlWorldImporter
			${PLUGIN_LIBS_DIR}/include/bullet/ConvexDecomposition
			${PLUGIN_LIBS_DIR}/include/bullet/GIMPACTUtils
			${PLUGIN_LIBS_DIR}/include/bullet/HACD
			${PLUGIN_LIBS_DIR}/include/bullet/InverseDynamics
			${PLUGIN_LIBS_DIR}/include/bullet/LinearMath
			${PLUGIN_LIBS_DIR}/include/GL
			${PLUGIN_LIBS_DIR}/include/openvr
	)
endif()


##### PLUGIN DEPENDENCY LOCATIONS & DLL COPYING (IF ANY) #####
if(GIP_BULLET_FROM_SOURCE)
	#Bullet is compiled from source, see above
elseif(WIN32)
	# Automatically copy all prebuilt runtime DLLs to application binary directory
	file(GLOB PLUGIN_DLLS
		"${PLUGIN_LIBS_DIR}/bin/*.dll"
		"${PLUGIN_PREBUILTS_DIR}/bin/*.dll"
	)
	if(PLUGIN_DLLS)
		file(COPY ${PLUGIN_DLLS} DESTINATION ${CMAKE_BINARY_DIR})
	endif()

	if(EXISTS "${PLUGIN_LIBS_DIR}/lib/win32")
		list(APPEND PLUGIN_LINKDIRS "${PLUGIN_LIBS_DIR}/lib/win32")
		list(APPEND PLUGIN_LIBDIRS "${PLUGIN_LIBS_DIR}/lib/win32")
	endif()
	if(EXISTS "${PLUGIN_LIBS_DIR}/bin")
		list(APPEND PLUGIN_LINKDIRS "${PLUGIN_LIBS_DIR}/bin")
	endif()

	if(EXISTS "${PLUGIN_PREBUILTS_DIR}/lib/win32")
		list(APPEND PLUGIN_LINKDIRS "${PLUGIN_PREBUILTS_DIR}/lib/win32")
		list(APPEND PLUGIN_LIBDIRS "${PLUGIN_PREBUILTS_DIR}/lib/win32")
	endif()
	if(EXISTS "${PLUGIN_PREBUILTS_DIR}/bin")
		list(APPEND PLUGIN_LINKDIRS "${PLUGIN_PREBUILTS_DIR}/bin")
	endif()
elseif(LINUX)
	#Linux libs here
elseif(APPLE)
	#Macos libs here
	find_package(Bullet REQUIRED)
	message(STATUS "Bullet found: ${BULLET_FOUND}")
	message(STATUS "Bullet include dirs: ${BULLET_INCLUDE_DIRS}")
	message(STATUS "Bullet libraries: ${BULLET_LIBRARIES}")
	list(APPEND PLUGIN_INCLUDES ${BULLET_INCLUDE_DIRS})
	list(APPEND PLUGIN_LINKLIBS ${BULLET_LIBRARIES})
endif()


##### PLUGIN DEPENDENCY LINKING (IF ANY) #####
if(GIP_BULLET_FROM_SOURCE)
	list(APPEND PLUGIN_LINKLIBS
				#Lib names
				BulletDynamics
				BulletCollision
				LinearMath
	)
elseif(WIN32)
	list(APPEND PLUGIN_LINKLIBS
				#Lib names
				LinearMath
				Bullet3Common
				BulletInverseDynamics
				BulletCollision
				BulletDynamics
				BulletSoftBody
				Bullet2FileLoader
				Bullet3Collision
				Bullet3Dynamics
				Bullet3Geometry
				Bullet3OpenCL_clew
				BulletExampleBrowserLib
				BulletFileLoader
				BulletInverseDynamicsUtils
				BulletRobotics
				BulletWorldImporter
				BulletXmlWorldImporter
				BussIK
				clsocket
				ConvexDecomposition
				freeglut
				GIMPACTUtils
				gwen
				HACD
				openvr_api
	)
elseif(LINUX)
	list(APPEND PLUGIN_LINKLIBS
				#Lib names
	)
elseif(APPLE)
	list(APPEND PLUGIN_LINKLIBS
				#Lib names
	)
endif()
