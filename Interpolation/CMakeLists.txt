add_executable(interpolate
#        mainAdam.cpp
        main.cpp
        interp.cpp
        )

target_link_libraries(interpolate common_options_target)
set_target_properties(interpolate PROPERTIES FOLDER "Examples")

