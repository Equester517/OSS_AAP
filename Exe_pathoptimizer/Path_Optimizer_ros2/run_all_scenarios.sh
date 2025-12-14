#!/bin/bash

# Run both Path_Optimizer scenarios and save outputs

echo "###################################################################"
echo "#  Path Optimizer - Running Both Scenarios"
echo "###################################################################"

cd /home/bskang/autoware/src/universe/autoware.universe/planning/Path_Optimizer

# Check if executable exists
if [ ! -f "build/path_optimizer" ]; then
    echo "Error: Executable not found at build/path_optimizer"
    echo ""
    echo "Building Path_Optimizer..."
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    cd ..
    
    if [ ! -f "build/path_optimizer" ]; then
        echo "Build failed!"
        exit 1
    fi
    echo "Build completed!"
    echo ""
fi

# ===================================================================
# Scenario 1: NO OBJECT
# ===================================================================
echo ""
echo "==================================================================="
echo "SCENARIO 1: NO OBJECT (wide lane, no obstacles)"
echo "==================================================================="
echo "Input files:"
echo "  - Path: test_files_in_standalone/input_path_no_object.csv"
echo "  - Odometry: test_files_in_standalone/input_odometry.csv"
echo "  - Left bound: test_files_in_standalone/no_object_left_bound.csv (x=-2.0m)"
echo "  - Right bound: test_files_in_standalone/no_object_right_bound.csv (x=2.0m)"
echo "  - Lane width: 4.0m"
echo ""

./build/path_optimizer \
    test_files_in_standalone/input_path_no_object.csv \
    test_files_in_standalone/input_odometry.csv \
    test_files_in_standalone/no_object_left_bound.csv \
    test_files_in_standalone/no_object_right_bound.csv

if [ $? -eq 0 ]; then
    cp optimized_trajectory.csv out_no_object_standalone.csv
    cp optimized_trajectory.csv test_files_in_standalone/out_no_object_standalone.csv
    echo ""
    echo "✓ NO OBJECT scenario completed!"
    echo "  Output saved to:"
    echo "    - out_no_object_standalone.csv"
    echo "    - test_files_in_standalone/out_no_object_standalone.csv"
    echo ""
else
    echo ""
    echo "✗ NO OBJECT scenario failed!"
    exit 1
fi

# ===================================================================
# Scenario 2: WITH OBJECT
# ===================================================================
echo ""
echo "==================================================================="
echo "SCENARIO 2: WITH OBJECT (narrow lane at y=25-35m)"
echo "==================================================================="
echo "Input files:"
echo "  - Path: test_files_in_standalone/input_path_with_object.csv"
echo "  - Odometry: test_files_in_standalone/input_odometry.csv"
echo "  - Left bound: test_files_in_standalone/with_object_left_bound.csv (x=-0.3m at y=25-35m)"
echo "  - Right bound: test_files_in_standalone/with_object_right_bound.csv (x=2.0m)"
echo "  - Lane width: 4.0m → 2.3m (object zone)"
echo ""

./build/path_optimizer \
    test_files_in_standalone/input_path_with_object.csv \
    test_files_in_standalone/input_odometry.csv \
    test_files_in_standalone/with_object_left_bound.csv \
    test_files_in_standalone/with_object_right_bound.csv

if [ $? -eq 0 ]; then
    cp optimized_trajectory.csv out_with_object_standalone.csv
    cp optimized_trajectory.csv test_files_in_standalone/out_with_object_standalone.csv
    echo ""
    echo "✓ WITH OBJECT scenario completed!"
    echo "  Output saved to:"
    echo "    - out_with_object_standalone.csv"
    echo "    - test_files_in_standalone/out_with_object_standalone.csv"
    echo ""
else
    echo ""
    echo "✗ WITH OBJECT scenario failed!"
    exit 1
fi

# ===================================================================
# Scenario 3: STD (Autoware elastic_band_smoother data)
# ===================================================================
echo ""
echo "==================================================================="
echo "SCENARIO 3: STD (Real Autoware data from elastic_band_smoother)"
echo "==================================================================="
echo "Input files:"
echo "  - Path: test_files_in_standalone/input_path_std.csv (126 points)"
echo "  - Odometry: test_files_in_standalone/input_odometry_std.csv"
echo "  - Left bound: test_files_in_standalone/std_left_bound.csv (41 points)"
echo "  - Right bound: test_files_in_standalone/std_right_bound.csv (38 points)"
echo "  - Data source: elastic_band_smoother.txt"
echo ""

./build/path_optimizer \
    test_files_in_standalone/input_path_std.csv \
    test_files_in_standalone/input_odometry_std.csv \
    test_files_in_standalone/std_left_bound.csv \
    test_files_in_standalone/std_right_bound.csv

if [ $? -eq 0 ]; then
    cp optimized_trajectory.csv out_std_standalone.csv
    cp optimized_trajectory.csv test_files_in_standalone/out_std_standalone.csv
    echo ""
    echo "✓ STD scenario completed!"
    echo "  Output saved to:"
    echo "    - out_std_standalone.csv"
    echo "    - test_files_in_standalone/out_std_standalone.csv"
    echo ""
else
    echo ""
    echo "✗ STD scenario failed!"
    exit 1
fi

# ===================================================================
# Summary
# ===================================================================
echo ""
echo "###################################################################"
echo "#  All Scenarios Completed!"
echo "###################################################################"
echo ""
echo "Output files created:"
echo "  Current directory:"
echo "    1. out_no_object_standalone.csv"
echo "    2. out_with_object_standalone.csv"
echo "    3. out_std_standalone.csv"
echo ""
echo "  Test directory (test_files_in_standalone/):"
echo "    1. out_no_object_standalone.csv"
echo "    2. out_with_object_standalone.csv"
echo "    3. out_std_standalone.csv"
echo ""
echo "Next steps:"
echo "  1. Compare the two synthetic scenarios:"
echo "     python3 compare_trajectories.py out_no_object_standalone.csv out_with_object_standalone.csv"
echo ""
echo "  2. Compare STD with ROS2 path_optimizer result:"
echo "     python3 compare_trajectories.py out_std_standalone.csv ../autoware_path_optimizer/out_new.txt"
echo ""
echo "  3. Compare with ROS2 results (synthetic scenarios):"
echo "     python3 compare_trajectories.py test_files_in_ros2/out_no_object.txt out_no_object_standalone.csv"
echo "     python3 compare_trajectories.py test_files_in_ros2/out_with_object.txt out_with_object_standalone.csv"
echo ""
echo "Expected results:"
echo "  - NO_OBJECT vs WITH_OBJECT: Object causes 40-120cm deviation"
echo "  - STD standalone vs ROS2: Should match elastic_band_smoother optimization"
echo ""
