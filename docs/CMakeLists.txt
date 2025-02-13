find_package(Doxygen REQUIRED doxygen OPTIONAL_COMPONENTS dot)

if (Doxygen_FOUND)
    message(STATUS "Doxygen executable: ${DOXYGEN_EXECUTABLE}")
else ()
    message(FATAL_ERROR "Doxygen not found!")
endif ()

set(DOXYGEN_INPUT_DIR_1 "${PROJECT_SOURCE_DIR}/include")
set(DOXYGEN_INPUT_DIR_2 "${PROJECT_SOURCE_DIR}/src")
set(DOXYGEN_INPUT_DIR_3 "${PROJECT_SOURCE_DIR}/README.md")

set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

set(DOXYFILE_IN "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in")
set(DOXYFILE_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")

set(DOXYFILE_MAIN_PAGE "${PROJECT_SOURCE_DIR}/README.md")

#file(APPEND ${DOXYFILE_OUT} "INPUT += ${CMAKE_SOURCE_DIR}/README.md\n")

configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)
message(STATUS "Doxygen build started...")

#add_custom_command(OUTPUT ${DOXYGEN_INDEX_FILE}
#        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
#        MAIN_DEPENDENCY ${DOXYFILE_IN} ${DOXYFILE_OUT}
#        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#        COMMENT "Generating API documentation with Doxygen"
#        VERBATIM
#)

add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
        DEPENDS ${DOXYFILE_OUT} ${DOXYGEN_INPUT_DIR_1} ${DOXYGEN_INPUT_DIR_2} ${DOXYGEN_INPUT_DIR_3}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
)

#doxygen_add_docs(docs
#        "${CMAKE_SOURCE_DIR}/README.md"
#        "${CMAKE_SOURCE_DIR}/include"
#        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")
