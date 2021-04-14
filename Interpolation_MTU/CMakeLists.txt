#SET(MTUSpline_SOURCES
#	MTUSpline.cpp
#	MTUSplineBase.cpp
#	SplineBasisFunction.cpp
#	MTUSplineData.cpp
#	MTUSplineDataRead.cpp
#	MTUSplineInterface.cpp
#)

#ADD_LIBRARY(MTUSpline STATIC
#  ${MTUSpline_SOURCES}
#)

#TARGET_LINK_LIBRARIES(MTUSpline
#	xmlInput
#	${XERCES_LIBRARY}
#	${PLATFORM_LIBS}
#	${Boost_CHRONO_LIBRARY}
#	${Boost_TIMER_LIBRARY}
#	${Boost_SYSTEM_LIBRARY}
#)

add_executable(interpolateMTU
        main.cpp
        MTUSpline.cpp
        MTUSplineBase.cpp
        SplineBasisFunction.cpp
        MTUSplineData.cpp
        MTUSplineDataRead.cpp
        MTUSplineInterface.cpp
)

target_link_libraries(interpolateMTU common_options_target)
set_target_properties(interpolateMTU PROPERTIES FOLDER "Examples")
