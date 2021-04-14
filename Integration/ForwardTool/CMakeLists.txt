add_executable(ForwardTool
        main.cpp
        )

target_link_libraries(ForwardTool common_options_target)
set_target_properties(ForwardTool PROPERTIES FOLDER "Examples")

