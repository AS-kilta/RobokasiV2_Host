add_subdirectory(kinematics)
set(KINEMATICS_HEADERS ${KINEMATICS_HEADERS} PARENT_SCOPE)

add_subdirectory(gui)
set(GUI_HEADERS ${GUI_HEADERS} PARENT_SCOPE)

add_subdirectory(hwio)
set(HWIO_HEADERS ${HWIO_HEADERS} PARENT_SCOPE)
