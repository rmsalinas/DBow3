INCLUDE_DIRECTORIES(. ./utils/)

FILE(GLOB hdrs_base "*.h" )
FILE(GLOB srcs_base "*.c*")

FILE(GLOB hdrs ${hdrs_base}  )
FILE(GLOB srcs  ${srcs_base} )


ADD_LIBRARY(${PROJECT_NAME} ${srcs} ${hdrs})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} )

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES 				# create *nix style library versions + symbolic links
        DEFINE_SYMBOL DBOW_DSO_EXPORTS
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_SOVERSION}
	CLEAN_DIRECT_OUTPUT 1 					# allow creating static and shared libs without conflicts
	OUTPUT_NAME "${PROJECT_NAME}${PROJECT_DLLVERSION}" 	# avoid conflicts between library and binary target names
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES} )

target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/src>
)

include(GNUInstallDirs)
install(
	TARGETS
	${PROJECT_NAME}
	EXPORT
	${PROJECT_NAME}Targets
	LIBRARY DESTINATION
	${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION
	${CMAKE_INSTALL_BINDIR}
	ARCHIVE DESTINATION
	${CMAKE_INSTALL_LIBDIR}
	INCLUDES DESTINATION
	include
	PUBLIC_HEADER DESTINATION
	include
)

install(
	EXPORT
	${PROJECT_NAME}Targets
	FILE
	${PROJECT_NAME}Targets.cmake
	NAMESPACE
	${PROJECT_NAME}::
	DESTINATION
	${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
  ${hdrs_base}
  DESTINATION
  include/${PROJECT_NAME}
)