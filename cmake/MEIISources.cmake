set(SRC_MAHI_EXO_II
        "${CMAKE_SOURCE_DIR}/src/MEII/MahiExoII/Exo.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/MahiExoII/MahiExoII.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/MahiExoII/MeiiConfiguration.cpp"
)

set(SRC_CONTROL
        "${CMAKE_SOURCE_DIR}/src/MEII/Control/DisturbanceObserver.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Control/DynamicMotionPrimitive.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Control/MinimumJerk.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Control/Trajectory.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Control/WayPoint.cpp"
)

set(SRC_UTILITY
        "${CMAKE_SOURCE_DIR}/src/MEII/Utility/EigenConversions.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Utility/logging_util.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Utility/Matrix.cpp"
        "${CMAKE_SOURCE_DIR}/src/MEII/Utility/VirtualInput.cpp"
)

set(HEADER_MAHI_EXO_II
        "${CMAKE_SOURCE_DIR}/include/MEII/MahiExoII/Exo.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/MahiExoII/MahiExoII.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/MahiExoII/MeiiConfiguration.hpp"
)

set(HEADER_CONTROL
        "${CMAKE_SOURCE_DIR}/include/MEII/Control/DisturbanceObserver.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Control/DynamicMotionPrimitive.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Control/MinimumJerk.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Control/Trajectory.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Control/WayPoint.hpp"
)

set(HEADER_UTILITY
        "${CMAKE_SOURCE_DIR}/include/MEII/Utility/EigenConversions.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Utility/logging_util.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Utility/Matrix.hpp"
        "${CMAKE_SOURCE_DIR}/include/MEII/Utility/VirtualInput.hpp"
)

# MEII expansion library source files
set(MEII_SOURCE_FILES
    ${SRC_MAHI_EXO_II}
    ${SRC_CONTROL}
    ${SRC_UTILITY}
)

set(MEII_HEADER_FILES
    ${HEADER_MAHI_EXO_II}
    ${HEADER_CONTROL}
    ${HEADER_UTILITY}
)