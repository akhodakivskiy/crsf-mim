cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "../components")

set(TEST_COMPONENTS "nav" CACHE STRING "List of components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(test_app)
