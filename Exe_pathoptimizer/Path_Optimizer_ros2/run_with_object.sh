#!/bin/bash

# Run Path_Optimizer with with_object scenario

echo "==================================================================="
echo "Running Path_Optimizer - WITH OBJECT scenario"
echo "==================================================================="

cd /home/bskang/autoware/src/universe/autoware.universe/planning/Path_Optimizer

# Check if executable exists
if [ ! -f "build/path_optimizer" ]; then
    echo "Error: Executable not found at build/path_optimizer"
    echo "Please build the project first:"
    echo "  cd /home/bskang/autoware/src/universe/autoware.universe/planning/Path_Optimizer"
    echo "  mkdir -p build && cd build"
    echo "  cmake .."
    echo "  make"
    exit 1
fi

# Run with with_object scenario CSV files
./build/path_optimizer \
  test_files_in_standalone/input_path_with_object.csv \
  test_files_in_standalone/input_odometry.csv \
  test_files_in_standalone/with_object_left_bound.csv \
  test_files_in_standalone/with_object_right_bound.csv# Check if optimization was successful
if [ $? -eq 0 ]; then
    # Save output to test_files_in_standalone directory
    cp optimized_trajectory.csv test_files_in_standalone/out_with_object_standalone.csv
    
    echo ""
    echo "==================================================================="
    echo "✓ WITH OBJECT scenario completed successfully!"
    echo "==================================================================="
    echo "Output saved to:"
    echo "  - optimized_trajectory.csv (current directory)"
    echo "  - test_files_in_standalone/out_with_object_standalone.csv"
    echo ""
else
    echo ""
    echo "==================================================================="
    echo "✗ WITH OBJECT scenario failed!"
    echo "==================================================================="
    exit 1
fi
