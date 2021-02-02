add_executable(integrate
        main.cpp
        )

target_link_libraries(integrate common_options_target)
set_target_properties(integrate PROPERTIES FOLDER "Examples")

