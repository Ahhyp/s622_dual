# C4.3 S3 -- S3-T12, the R24 static guard (no new dependency: plain CMake script).
#
# R24 (C4.3_S0_审计报告.md:1141-1143) forbids the S3 adapter from feeding the QP the
# legacy object row, i.e. from calling back into the kinematics object APIs that S3
# replaces.  A runtime test cannot prove the absence of a call on a path that was
# never taken, so this is a source scan: it fails on any hit in src/ or include/.
#
# Deliberately NOT scanned: test/ and tools/.  The golden generator and the legacy
# golden test must call the legacy APIs -- that is how the bit-for-bit
# unchanged-behaviour evidence is produced.  Scanning them would make the guard
# impossible to satisfy instead of meaningful.
#
# Usage: cmake -DSRC_DIR=<dual_arm_transport_mock> -P check_no_legacy_object_api.cmake

if(NOT DEFINED SRC_DIR)
    message(FATAL_ERROR "SRC_DIR is required")
endif()

set(FORBIDDEN
    "worldTobject"
    "objectJacobianWorld"
    "objectJacobianBody"
    "objectError"
    "objectErrorJacobian")

set(scan_dirs "${SRC_DIR}/src" "${SRC_DIR}/include")
set(hits "")
set(scanned 0)

foreach(dir IN LISTS scan_dirs)
    file(GLOB_RECURSE files "${dir}/*.cpp" "${dir}/*.hpp" "${dir}/*.h")
    foreach(f IN LISTS files)
        file(READ "${f}" content)
        math(EXPR scanned "${scanned} + 1")
        foreach(sym IN LISTS FORBIDDEN)
            string(FIND "${content}" "${sym}" pos)
            if(NOT pos EQUAL -1)
                file(RELATIVE_PATH rel "${SRC_DIR}" "${f}")
                list(APPEND hits "${rel}: contains forbidden legacy object API '${sym}'")
            endif()
        endforeach()
    endforeach()
endforeach()

if(scanned EQUAL 0)
    message(FATAL_ERROR "S3-T12 scanned zero files under ${SRC_DIR}/{src,include} -- "
                        "the guard would pass vacuously")
endif()

if(hits)
    string(REPLACE ";" "\n  " pretty "${hits}")
    message(FATAL_ERROR "S3-T12 (R24) FAILED: S3 sources must not use the legacy "
                        "object APIs:\n  ${pretty}")
endif()

message(STATUS "S3-T12 (R24) OK: ${scanned} files scanned, zero legacy object API references")
