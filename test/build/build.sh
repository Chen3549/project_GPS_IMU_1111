#!/bin/bash

TARGET_DIR="unit_tests_build"  # Build directory
BINARY_NAME="unit_tests"       # Binary executable name
CMAKE_SRC_PATH="../../"         # Path to directory containing CMakeLists.txt (relative to build dir)

# ==================== Core Functions ====================
# Clean: Remove build directory and binary file in current directory
clean() {
    echo "=== Starting Cleanup ==="
    # Delete build directory if it exists
    if [ -d "${TARGET_DIR}" ]; then
        echo "Removing build directory: ${TARGET_DIR}"
        rm -rf "${TARGET_DIR}"
    else
        echo "Build directory ${TARGET_DIR} does not exist, no need to delete"
    fi

    # Delete binary file in current directory if it exists
    if [ -f "./${BINARY_NAME}" ]; then
        echo "Removing binary file: ./${BINARY_NAME}"
        rm -f "./${BINARY_NAME}"
    else
        echo "Binary file ./${BINARY_NAME} does not exist, no need to delete"
    fi
    echo "=== Cleanup Completed ==="
}

# Rebuild: Full recompilation (clean first, then build)
rebuild() {
    echo "=== Starting Full Rebuild ==="
    # Step 1: Clean old files
    clean
    # Step 2: Execute build (reuse build function logic)
    build
    echo "=== Full Rebuild Completed ==="
}

# Default build (incremental compilation)
build() {
    echo "=== Starting Compilation (Incremental) ==="
    # 1. Create build directory if it doesn't exist
    if [ ! -d "${TARGET_DIR}" ]; then
        echo "Creating build directory: ${TARGET_DIR}"
        mkdir -p "${TARGET_DIR}"
    fi

    # 2. Enter build directory (exit script if failed)
    echo "Entering build directory: ${TARGET_DIR}"
    cd "${TARGET_DIR}" || {
        echo "ERROR: Failed to enter directory ${TARGET_DIR}, compilation aborted"
        exit 1
    }

    # 3. Execute CMake (regenerates Makefile if CMakeCache doesn't exist or source changed)
    echo "Running CMake: cmake ${CMAKE_SRC_PATH}"
    cmake "${CMAKE_SRC_PATH}" || {
        echo "ERROR: CMake configuration failed, compilation aborted"
        exit 1
    }

    # 4. Execute make (incremental build: only compiles changed files)
    echo "Starting compilation: make"
    make || {
        echo "ERROR: Compilation failed, aborted"
        exit 1
    }

    # 5. Copy binary file to current directory (parent directory of build dir)
    if [ -f "./${BINARY_NAME}" ]; then
        echo "Copying binary file: ./${BINARY_NAME} -> ../${BINARY_NAME}"
        cp -f "./${BINARY_NAME}" "../${BINARY_NAME}"  # -f: force overwrite without prompt
    else
        echo "ERROR: Compilation did not generate binary file ${BINARY_NAME}"
        exit 1
    fi

    echo "=== Compilation Completed ==="
    echo "Binary file path: $(cd .. && pwd)/${BINARY_NAME}"
}

# ==================== Menu ====================
case "$1" in
    clean)
        # Execute cleanup: sh build.sh clean
        clean
        ;;
    rebuild)
        # Execute full rebuild: sh build.sh rebuild
        rebuild
        ;;
    "")
        # No argument: run default incremental build: sh build.sh
        build
        ;;
    *)
        # Invalid command prompt
        echo "ERROR: Invalid command!"
        echo "Supported commands:"
        echo "  sh $0          - Default incremental build"
        echo "  sh $0 clean    - Clean build directory and binary file"
        echo "  sh $0 rebuild  - Full rebuild (clean + build from scratch)"
        exit 1
        ;;
esac
