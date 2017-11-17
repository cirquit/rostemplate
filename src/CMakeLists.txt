# 'src' CMakeLists.txt

#############
#           #
# Libraries #
#           #
#############

# all library sources
set(SOURCES util.hpp)

# create a library
set(LIB_NAME shared_utils)
add_library(${LIB_NAME} ${SOURCES})
target_link_libraries(${LIB_NAME}) #{OTHER SOURCES}
set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)

###############
#             #
# Executables #
#             #
###############

# create exectuable 
add_executable(listener listener.cpp)
# add exectuable dependencies
target_link_libraries(listener ${catkin_LIBRARIES} ${LIB_NAME})

add_executable(forwarder forwarder.cpp)
target_link_libraries(forwarder ${catkin_LIBRARIES} ${LIB_NAME})

add_executable(publisher publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${LIB_NAME})