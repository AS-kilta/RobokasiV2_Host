# Add headers
file(GLOB SUB_HEADERS "*.hpp")

set(GUI_HEADERS
    ${GUI_HEADERS}
    ${SUB_HEADERS}
    PARENT_SCOPE)
