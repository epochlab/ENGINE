# Registers one ctest entry per check in a validator built on tools/check.h, by asking the binary itself what it
# contains. CMake cannot run a target at configure time, so this uses the same POST_BUILD mechanism as CMake's own
# gtest_discover_tests: the binary writes a CMake fragment, and TEST_INCLUDE_FILES pulls it into the test set.
# The consequence worth having is that adding a check adds a ctest entry with no CMake edit at all.
#
# ENGINE_TEST_THREADS keeps `ctest -j` from oversubscribing: every render-driving check would otherwise build a pool
# sized to hardware_concurrency(), so N concurrent checks would spawn N*cores threads on cores. The binary emits a
# matching PROCESSORS property so ctest's own job pool accounts for what each test will actually use.
function(engine_discover_checks target)
    set(fragment "${CMAKE_CURRENT_BINARY_DIR}/${target}_checks.cmake")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND "$<TARGET_FILE:${target}>" --list-ctest "--threads=${ENGINE_TEST_THREADS}" > "${fragment}"
        BYPRODUCTS "${fragment}"
        VERBATIM
        COMMENT "Discovering checks in ${target}")
    set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES "${fragment}")
endfunction()
