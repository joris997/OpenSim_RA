add_executable(interpolateTest
        main.cpp
        )

target_link_libraries(interpolateTest common_options_target)
set_target_properties(interpolateTest PROPERTIES FOLDER "Examples")
