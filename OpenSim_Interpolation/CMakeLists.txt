add_executable(OpenSim_Interpolation
        main.cpp
        )

target_link_libraries(OpenSim_Interpolation common_options_target)
set_target_properties(OpenSim_Interpolation PROPERTIES FOLDER "Examples")
