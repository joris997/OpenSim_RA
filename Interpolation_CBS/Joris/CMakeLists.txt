add_executable(interpolateCBS
        main.cpp
        SplineBasisFunction.cpp
        SplineData.cpp)

target_link_libraries(interpolateCBS common_options_target)
set_target_properties(interpolateCBS PROPERTIES FOLDER "Examples")
