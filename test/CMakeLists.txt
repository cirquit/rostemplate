# define all tests here

# single test
set(TEST_SRC_1 simple_test.cpp)
set(TEST_NAME_1 simple_test)
add_executable(${TEST_NAME_1} catch.h main.cpp ${TEST_SRC_1})
target_link_libraries(${TEST_NAME_1} ${LIB_NAME})