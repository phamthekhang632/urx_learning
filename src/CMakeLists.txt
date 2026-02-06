set(controller_SRC
  Ur10Learning.cpp
)

set(controller_HDR
  Ur10Learning.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DUr10Learning_EXPORTS")

