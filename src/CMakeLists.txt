add_subdirectory(app)
set(APP_SOURCES ${APP_SOURCES} PARENT_SCOPE)

add_subdirectory(kinematics)
set(KINEMATICS_SOURCES ${KINEMATICS_SOURCES} PARENT_SCOPE)

add_subdirectory(gui)
set(GUI_SOURCES ${GUI_SOURCES} PARENT_SCOPE)

add_subdirectory(hwio)
set(HWIO_SOURCES ${HWIO_SOURCES} PARENT_SCOPE)
