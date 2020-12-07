add_executable(momentArm
        main.cpp
        )

target_link_libraries(momentArm common_options_target)
set_target_properties(momentArm PROPERTIES FOLDER "Examples")
