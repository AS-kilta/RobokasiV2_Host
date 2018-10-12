# Add sources
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES)

# Add headers
file(GLOB SUB_HEADERS "*.hpp")

set(HWIO_SOURCES
    ${HWIO_SOURCES}
    ${SUB_HEADERS}
    ${SUB_SOURCES}
    PARENT_SCOPE)
