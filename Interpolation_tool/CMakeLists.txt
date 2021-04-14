add_executable(interpolateTool
        main.cpp
        )

target_link_libraries(interpolateTool common_options_target)
set_target_properties(interpolateTool PROPERTIES FOLDER "Examples")
