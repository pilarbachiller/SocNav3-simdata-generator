cmake_minimum_required(VERSION 2.8.11)

MACRO( ROBOCOMP_WRAP_ICE )
  # External Slice source paths
  SET (EXTERNAL_SLICE "")
  SET (SLICE_PATH "$ENV{SLICE_PATH};../interfaces;/opt/robocomp/interfaces;./ice_files/;")
  SET (INC_ROBOCOMPSLICE_PATH "true" )
  SET (ADDITIONAL_SLICE_INCLUDE_PATH "../../interfaces")
#  FOREACH (SPATH ${SLICE_PATH})
#     IF( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
#       SET(INC_ROBOCOMPSLICE_PATH "false")
#     ELSE( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
#       SET(ADDITIONAL_SLICE_INCLUDE_PATH ${ADDITIONAL_SLICE_INCLUDE_PATH} -I${SPATH})
#     ENDIF( ${RoboComp_INTERFACES_DIR} STREQUAL ${SPATH})
#  ENDFOREACH (SPATH ${SLICE_PATH})
#  IF (${INC_ROBOCOMPSLICE_PATH} STREQUAL "true")
#     SET (SLICE_PATH "${SLICE_PATH};${RoboComp_INTERFACES_DIR}")
#  ENDIF(${INC_ROBOCOMPSLICE_PATH} STREQUAL "true")
#  MESSAGE(STATUS "$SLICE_PATH=\"${SLICE_PATH}\"")
  FOREACH (SPATH ${SLICE_PATH})
    MESSAGE(STATUS "Adding ${SPATH} to the Slice directory set.")
    SET (EXTERNAL_SLICE "${EXTERNAL_SLICE} -I${SPATH} ")
  ENDFOREACH (SPATH)

  FOREACH( input_file ${ARGN} )
    SET (SLICE_FILE_FOUND "false")
	IF( input_file STREQUAL "Logger" )
		MESSAGE(STATUS "Compile with LoggerComp support." )
		ADD_DEFINITIONS(-DCOMPILE_LOGGERCOMP=1)
	ENDIF( input_file STREQUAL "Logger" )
    FOREACH (SPATH ${SLICE_PATH})
      IF (EXISTS "${SPATH}/${input_file}.ice")
        MESSAGE(STATUS "Adding rule to generate ${input_file}.cpp and ${input_file}.h from ${SPATH}/${input_file}.ice  (${SLICECPP_PATH}slice2cpp)" )
        ADD_CUSTOM_COMMAND (
          OUTPUT ${input_file}.cpp ${input_file}.h
          COMMAND ${SLICECPP_PATH}slice2cpp --underscore -I${RoboComp_INTERFACES_DIR} ${ADDITIONAL_SLICE_INCLUDE_PATH} -I. ../${SPATH}/${input_file}.ice --output-dir .
          DEPENDS ../${SPATH}/${input_file}.ice
          COMMENT "Generating ${input_file}.cpp and ${input_file}.h from ${input_file}.ice"
        )
        SET ( HEADERS ${HEADERS} ./${input_file}.h)        
        SET ( SOURCES ${SOURCES} ./${input_file}.cpp)
        SET_PROPERTY(SOURCE ${input_file}.cpp PROPERTY SKIP_AUTOGEN ON)
        SET ( SLICE_FILE_FOUND "true")
	break ()
      ENDIF (EXISTS "${SPATH}/${input_file}.ice")
    ENDFOREACH (SPATH)
    IF (${SLICE_FILE_FOUND} STREQUAL "false")
      MESSAGE(FATAL_ERROR "${input_file}.ice not found in any of the Slice directories (${SLICE_PATH}).")
    ENDIF (${SLICE_FILE_FOUND} STREQUAL "false")
  ENDFOREACH( input_file )
ENDMACRO( ROBOCOMP_WRAP_ICE )

#IF ( "$ENV{ROBOCOMP}" STREQUAL "")
#  MESSAGE(WARNING "ROBOCOMP variable not set. Using the default value: /opt/robocomp")
#  SET (ENV{ROBOCOMP} "/opt/robocomp/")
#ENDIF ( "$ENV{ROBOCOMP}" STREQUAL "")

#IF(NOT EXISTS $ENV{ROBOCOMP}/cmake)
#  MESSAGE(FATAL_ERROR "Couldn't find RoboComp in $ENV{ROBOCOMP}! Exiting...")
#ENDIF(NOT EXISTS $ENV{ROBOCOMP}/cmake)

#ROBOCOMP_WRAP_ICE( CommonBehavior GenericBase ObjectDetector OmniRobot PeopleDetector SNGNN2D WallDetector GoalPublisher)

#INCLUDE( $ENV{ROBOCOMP}/cmake/robocomp.cmake )

#find_package(Qt6 COMPONENTS Core REQUIRED)
INCLUDE_DIRECTORIES (
  ${CMAKE_CURRENT_BINARY_DIR}
  .
  ${CMAKE_BINARY_DIR}
)

INCLUDE ( CMakeListsSpecific.txt)

# Sources set
SET ( SOURCES
  ${SOURCES}
  #$ENV{ROBOCOMP}/classes/rapplication/rapplication.cpp
  #$ENV{ROBOCOMP}/classes/sigwatch/sigwatch.cpp
  #$ENV{ROBOCOMP}/classes/qlog/qlog.cpp
  main.cpp
  genericmonitor.cpp
  commonbehaviorI.cpp
  genericworker.cpp
  objectdetectorI.cpp
  peopledetectorI.cpp
  sngnn2dI.cpp
  walldetectorI.cpp
  goalpublisherI.cpp
)




#ROBOCOMP
#ROBOCOMP_INITIALIZE( $ENV{ROBOCOMP}/ )
ROBOCOMP_WRAP_ICE( CommonBehavior GenericBase ObjectDetector OmniRobot PeopleDetector SNGNN2D WallDetector GoalPublisher)
SET (EXECUTABLE_OUTPUT_PATH ${RC_COMPONENT_DEVEL_PATH}/bin)


# Specify construction and link process
ADD_EXECUTABLE( trajectory ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( trajectory ${LIBS} ${STATIC_LIBS} ${SPECIFIC_LIBS} ${QT_LIBRARIES} ${Ice_LIBRARIES})
#INSTALL(FILES ${EXECUTABLE_OUTPUT_PATH}/trajectory DESTINATION ${RC_COMPONENT_INSTALL_PATH}/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )

