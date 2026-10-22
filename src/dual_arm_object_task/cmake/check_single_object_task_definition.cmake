# C4.3 S4 W0 -- acceptance clause W0-1: the anchored object task must have exactly
# ONE definition, and the production packages must go through it instead of
# re-deriving the anchored frame math.
#
# Why a source scan and not a linker check: a second, hand-written copy of the
# four lines that build the object row would link perfectly and silently drift
# from the verified one.  Only the source says how many copies exist.
#
# Usage: cmake -DSRC_ROOT=<.../src> -P check_single_object_task_definition.cmake
#
# Exit: FATAL_ERROR on a violation, or on a vacuous pass (zero files scanned).

if(NOT DEFINED SRC_ROOT)
    message(FATAL_ERROR "SRC_ROOT is required")
endif()

set(ONE_OF_ME
    "class AnchoredObjectTaskBuilder"
    "struct AnchoredObjectTask ")

# Production packages that must NOT call the raw anchored-frame math directly:
# they are supposed to consume dual_arm_object_task::AnchoredObjectTaskBuilder.
# dual_arm_transport_mock is deliberately absent -- it is the reference
# implementation and its own audit path is allowed to call these.
set(PROD_PACKAGES
    "dual_arm_qp_controller"
    "dual_arm_transport_executor"
    "dual_arm_object_task_controller")
set(PROD_FORBIDDEN
    "anchoredObjectPose("
    "anchoredObjectTwistJacobian("
    "anchoredObjectErrorJacobian("
    "decoupledPoseError("
    "decoupledErrorRateMap(")

file(GLOB_RECURSE all_files "${SRC_ROOT}/*/*.hpp" "${SRC_ROOT}/*/*.cpp"
                           "${SRC_ROOT}/*/*.h")
if(NOT all_files)
    message(FATAL_ERROR "W0-1 scanned zero files under ${SRC_ROOT} -- "
                        "the guard would pass vacuously")
endif()

set(violations "")
set(count_builder 0)
set(count_task 0)
set(nfiles 0)

foreach(f IN LISTS all_files)
    math(EXPR nfiles "${nfiles} + 1")
    file(READ "${f}" content)
    file(RELATIVE_PATH rel "${SRC_ROOT}" "${f}")

    if(rel MATCHES "^dual_arm_object_task/")
        string(FIND "${content}" "class AnchoredObjectTaskBuilder" p_builder)
        string(FIND "${content}" "struct AnchoredObjectTask " p_task)
        if(NOT p_builder EQUAL -1)
            math(EXPR count_builder "${count_builder} + 1")
            list(APPEND where_builder "${rel}")
        endif()
        if(NOT p_task EQUAL -1)
            math(EXPR count_task "${count_task} + 1")
            list(APPEND where_task "${rel}")
        endif()
    endif()

    foreach(pkg IN LISTS PROD_PACKAGES)
        if(rel MATCHES "^${pkg}/")
            foreach(sym IN LISTS PROD_FORBIDDEN)
                string(FIND "${content}" "${sym}" pos)
                if(NOT pos EQUAL -1)
                    list(APPEND violations
                         "${rel}: production code calls '${sym}' directly; "
                         "use dual_arm_object_task::AnchoredObjectTaskBuilder")
                endif()
            endforeach()
        endif()
    endforeach()
endforeach()

if(NOT count_builder EQUAL 1)
    string(REPLACE ";" ", " pretty "${where_builder}")
    list(APPEND violations
         "expected exactly 1 'class AnchoredObjectTaskBuilder', found ${count_builder} (${pretty})")
endif()
if(NOT count_task EQUAL 1)
    string(REPLACE ";" ", " pretty "${where_task}")
    list(APPEND violations
         "expected exactly 1 'struct AnchoredObjectTask', found ${count_task} (${pretty})")
endif()

if(violations)
    string(REPLACE ";" "\n  " pretty "${violations}")
    message(FATAL_ERROR "W0-1 FAILED (single definition of the anchored object task):\n  ${pretty}")
endif()

message(STATUS "W0-1 OK: ${nfiles} files scanned; exactly one AnchoredObjectTaskBuilder "
               "and one AnchoredObjectTask; no production package bypasses the builder")
