# Add headers
file(GLOB SUB_HEADERS "*.hpp")

set(HWIO_HEADERS
    ${HWIO_HEADERS}
    ${SUB_HEADERS}
    PARENT_SCOPE)
