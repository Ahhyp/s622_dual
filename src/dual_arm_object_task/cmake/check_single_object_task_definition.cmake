# C4.3 S4 W0 -- acceptance clause W0-1: the anchored object task must have exactly
# ONE definition, and no production package may re-derive the anchored frame math.
#
# Why a source scan and not a linker check: a second, hand-written copy of the
# four lines that build the object row would link perfectly and silently drift
# from the verified one.  Only the source says how many copies exist.
#
# Usage: cmake -DSRC_ROOT=<.../src> -P check_single_object_task_definition.cmake
# Exit:  non-zero (FATAL_ERROR) on any violation, on a vacuous scan, or when the
#        canonical definition is missing -- i.e. a failure can never look like a
#        pass.
#
# WHAT THIS RULE ACTUALLY CHECKS (written out so the rule is not trusted beyond
# what it does):
#   1. `class AnchoredObjectTaskBuilder` / `struct AnchoredObjectTask` are
#      *defined* (not merely declared) exactly once, and that one definition is
#      inside dual_arm_object_task/.
#   2. Any such definition outside dual_arm_object_task/ is a violation, in any
#      package under SRC_ROOT -- not only in a hard-coded list.  (The first
#      version of this script counted only inside dual_arm_object_task/, so a
#      copy in another package passed; that hole is what this rewrite closes.)
#   3. The raw anchored-frame math (anchoredObjectPose / anchoredObjectTwistJacobian /
#      anchoredObjectErrorJacobian / decoupledPoseError / decoupledErrorRateMap)
#      may only be called from the packages listed in MATH_EXEMPT.
#   4. Comments are stripped and whitespace is collapsed before matching, so a
#      comment, a `#include`, or a forward declaration never trips the guard.
#
# WHAT IT DOES NOT DO (detection boundary -- do not read more into a PASS):
#   * It is a TEXT rule, not semantic analysis: no macro expansion, no template
#     instantiation, no generated code, no cross-file data flow.
#   * A copy that re-implements the same numbers without using any of the five
#     canonical entry points (e.g. by re-deriving logSO3/adjoint from scratch)
#     is NOT detected.  Only the review of a new package can catch that.
#   * `git`-ignored or outside-SRC_ROOT code is not scanned.
#   * Rule 3 restricts *calls to the S1/S3 entry points*; it is not a proof that
#     a package does no duplicated kinematics work at all.

if(NOT DEFINED SRC_ROOT)
    message(FATAL_ERROR "SRC_ROOT is required")
endif()

set(CANON_PKG "dual_arm_object_task")
set(CANON_INCLUDE_REL "${CANON_PKG}/include/dual_arm_object_task/anchored_object_task.hpp")

# Packages allowed to call the raw frame math directly:
#   dual_arm_kinematics      -- it DEFINES those entry points (object_frame.hpp)
#   dual_arm_object_task     -- the single canonical implementation of the task
#   dual_arm_transport       -- frozen S2 candidate screening (anchored pose only,
#                               no Jacobian, no error, no QP row; S2 is frozen)
#   dual_arm_transport_mock  -- the S3 reference implementation and its audit
#                               tests (S3-T2/T8/T21 exercise the raw math on purpose)
set(MATH_EXEMPT
    "dual_arm_kinematics"
    "dual_arm_object_task"
    "dual_arm_transport"
    "dual_arm_transport_mock")

set(RAW_MATH
    "anchoredObjectPose("
    "anchoredObjectTwistJacobian("
    "anchoredObjectErrorJacobian("
    "decoupledPoseError("
    "decoupledErrorRateMap(")

# A definition, not a declaration: the name is followed by `{` (or by a base
# class list, or by `final`).  `class AnchoredObjectTaskBuilder;` and
# `struct AnchoredObjectTask;` therefore do not match.
set(RE_DEF_BUILDER "class[ ]+AnchoredObjectTaskBuilder[ ]*(final[ ]*)?[{:]")
set(RE_DEF_TASK "struct[ ]+AnchoredObjectTask[ ]*(final[ ]*)?[{:]")

if(NOT IS_DIRECTORY "${SRC_ROOT}")
    message(FATAL_ERROR "W0-1: SRC_ROOT '${SRC_ROOT}' is not a directory -- refusing to report a pass")
endif()

if(NOT EXISTS "${SRC_ROOT}/${CANON_INCLUDE_REL}")
    message(FATAL_ERROR
        "W0-1: the canonical definition file '${SRC_ROOT}/${CANON_INCLUDE_REL}' is missing.\n"
        "A scan without it cannot prove uniqueness, so it is treated as a failure "
        "(a missing file must never look like a pass).")
endif()

file(GLOB_RECURSE all_files
     "${SRC_ROOT}/*/*.hpp" "${SRC_ROOT}/*/*.cpp" "${SRC_ROOT}/*/*.h")
list(LENGTH all_files nfiles)
if(nfiles LESS 2)
    message(FATAL_ERROR "W0-1 scanned ${nfiles} file(s) under ${SRC_ROOT} -- "
                        "the guard would pass vacuously")
endif()

set(count_builder 0)
set(count_task 0)
set(violations "")
set(packages "")
set(ncomment_only 0)

foreach(f IN LISTS all_files)
    file(READ "${f}" content)
    file(RELATIVE_PATH rel "${SRC_ROOT}" "${f}")
    string(REGEX MATCH "^[^/]+" pkg "${rel}")
    list(APPEND packages "${pkg}")

    # Strip // and /* */ comments, then collapse whitespace, so that neither a
    # comment nor the layout of the code can change the verdict.
    string(REGEX REPLACE "//[^\n]*" "" code "${content}")
    string(REGEX REPLACE "/\\*[^*]*\\*+([^/*][^*]*\\*+)*/" "" code "${code}")
    string(REGEX REPLACE "[ \t\r\n]+" " " code "${code}")

    string(REGEX MATCH "${RE_DEF_BUILDER}" m_builder "${code}")
    string(REGEX MATCH "${RE_DEF_TASK}" m_task "${code}")

    if(pkg STREQUAL CANON_PKG)
        if(NOT m_builder STREQUAL "")
            math(EXPR count_builder "${count_builder} + 1")
            list(APPEND where_builder "${rel}")
        endif()
        if(NOT m_task STREQUAL "")
            math(EXPR count_task "${count_task} + 1")
            list(APPEND where_task "${rel}")
        endif()
    else()
        # Rule 2: a second definition anywhere else is a violation, whatever the
        # package is called.  This is the hole the first version left open.
        if(NOT m_builder STREQUAL "")
            list(APPEND violations
                 "${rel}: second definition of 'AnchoredObjectTaskBuilder' outside ${CANON_PKG}/; "
                 "the anchored object task must have exactly one implementation")
        endif()
        if(NOT m_task STREQUAL "")
            list(APPEND violations
                 "${rel}: second definition of 'AnchoredObjectTask' outside ${CANON_PKG}/; "
                 "the anchored object task must have exactly one implementation")
        endif()
    endif()

    # Rule 3: raw frame math is only allowed in the exempt packages.
    list(FIND MATH_EXEMPT "${pkg}" exempt_idx)
    if(exempt_idx EQUAL -1)
        foreach(sym IN LISTS RAW_MATH)
            string(FIND "${code}" "${sym}" pos)
            if(NOT pos EQUAL -1)
                list(APPEND violations
                     "${rel}: package '${pkg}' calls '${sym}' directly; "
                     "use dual_arm_object_task::AnchoredObjectTaskBuilder")
            endif()
        endforeach()
    endif()
endforeach()

if(NOT count_builder EQUAL 1)
    string(REPLACE ";" ", " pretty "${where_builder}")
    list(APPEND violations
         "expected exactly 1 definition of 'class AnchoredObjectTaskBuilder', found ${count_builder} (${pretty})")
endif()
if(NOT count_task EQUAL 1)
    string(REPLACE ";" ", " pretty "${where_task}")
    list(APPEND violations
         "expected exactly 1 definition of 'struct AnchoredObjectTask', found ${count_task} (${pretty})")
endif()

list(REMOVE_DUPLICATES packages)
list(LENGTH packages npackages)

if(violations)
    string(REPLACE ";" "\n  " pretty "${violations}")
    message(FATAL_ERROR
        "W0-1 FAILED (single definition of the anchored object task):\n  ${pretty}")
endif()

message(STATUS "W0-1 OK: ${nfiles} files / ${npackages} packages scanned under ${SRC_ROOT}; "
               "exactly one AnchoredObjectTaskBuilder and one AnchoredObjectTask, "
               "both in ${CANON_PKG}/; no other package calls the raw frame math")
