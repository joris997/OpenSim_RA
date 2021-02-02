add_executable(CPodes
        main.cpp
        )

target_link_libraries(CPodes common_options_target)
set_target_properties(CPodes PROPERTIES FOLDER "Examples")

