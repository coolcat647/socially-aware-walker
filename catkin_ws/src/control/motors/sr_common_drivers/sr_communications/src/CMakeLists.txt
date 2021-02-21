SET(DIRECTORIES ipc serial_port)

FOREACH(DIR ${DIRECTORIES})
    add_subdirectory(${DIR})
ENDFOREACH(DIR)
