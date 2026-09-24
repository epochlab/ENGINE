# cmake -P script: writes PATHTRACER_GIT_SHA to OUTPUT, rewritten only when the SHA changes (LLVM's GenerateVersionFromVCS pattern).
set(sha "unknown")
# EXISTS also matches a .git file, which is what a submodule has; a tarball checkout reports "unknown" rather than failing the build.
if(GIT_EXECUTABLE AND EXISTS "${SOURCE_DIR}/.git")
    execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
                    WORKING_DIRECTORY "${SOURCE_DIR}"
                    OUTPUT_VARIABLE sha OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
                    WORKING_DIRECTORY "${SOURCE_DIR}"
                    OUTPUT_VARIABLE dirty OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT dirty STREQUAL "")
        set(sha "${sha}+dirty")
    endif()
endif()

file(WRITE "${OUTPUT}.tmp" "#pragma once\n#define PATHTRACER_GIT_SHA \"${sha}\"\n")
file(COPY_FILE "${OUTPUT}.tmp" "${OUTPUT}" ONLY_IF_DIFFERENT)
file(REMOVE "${OUTPUT}.tmp")
