# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mics6814)

set_source_files_properties(tags PROPERTIES GENERATED true)
add_custom_target(
    tags
    COMMAND ctags -R --languages=C --exclude=build
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_dependencies(__idf_mics6814 tags)
