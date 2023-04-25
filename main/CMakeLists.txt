set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers")
idf_component_register(SRCS main.cpp ultrasonic.cpp ultrasonic.h
        INCLUDE_DIRS ".")
