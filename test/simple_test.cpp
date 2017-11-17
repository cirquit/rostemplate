#include "catch.h"
#include "../src/util.hpp"

TEST_CASE("simple-test", "[simple]")
{
    REQUIRE(foo() == 1);
}
