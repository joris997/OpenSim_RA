add_executable(randomTest
        main.cpp
        )

target_link_libraries(randomTest common_options_target)
set_target_properties(randomTest PROPERTIES FOLDER "Examples")
