# Registers one ctest entry per tools/check.h check by asking the binary itself.
function(pathtracer_discover_checks target)
    set(fragment "${CMAKE_CURRENT_BINARY_DIR}/${target}_checks.cmake")
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND "$<TARGET_FILE:${target}>" --list-ctest "--threads=${PATHTRACER_TEST_THREADS}" > "${fragment}"
        BYPRODUCTS "${fragment}"
        VERBATIM
        COMMENT "Discovering checks in ${target}")
    set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES "${fragment}")
endfunction()
