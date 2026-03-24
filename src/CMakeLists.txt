set(controller_SRC UrXLearning.cpp)

set(controller_HDR UrXLearning.h)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
                                                 "-DUrXLearning_EXPORTS")
