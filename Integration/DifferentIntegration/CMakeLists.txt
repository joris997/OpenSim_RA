add_executable(DifferentIntegration
        main.cpp
        )

target_link_libraries(DifferentIntegration common_options_target)
set_target_properties(DifferentIntegration PROPERTIES FOLDER "Examples")

