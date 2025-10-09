#!/bin/bash

# SPG Integration Test Visualization Script
# Usage: ./visualize_tests.sh [test_name]

cd "$(dirname "$0")/build/test"

echo "=== SPG Integration Test Visualization ==="
echo ""

if [ $# -eq 0 ]; then
    echo "Available visualization tests:"
    echo "  1. wall       - 5 obstacles in a horizontal line"
    echo "  2. moving     - 8 moving obstacles with bouncing physics"  
    echo "  3. edge       - Robot navigation near field boundaries"
    echo "  4. all        - Run all visualization tests"
    echo ""
    echo "Usage: $0 [wall|moving|edge|all]"
    echo "   or: SPG_VISUALIZE=1 ./test_static_obstacles_integration --gtest_filter=*Visualize*"
    exit 0
fi

export SPG_VISUALIZE=1

case "$1" in
    "wall")
        echo "Running Obstacle Wall visualization test..."
        echo "You should see 5 obstacles in a horizontal line blocking the robot's path."
        ./test_static_obstacles_integration --gtest_filter=*VisualizeObstacleWall*
        ;;
    "moving")
        echo "Running Moving Obstacles visualization test..."
        echo "You should see obstacles moving in circular patterns and bouncing off walls."
        ./test_static_obstacles_integration --gtest_filter=*VisualizeMoving*
        ;;
    "edge")
        echo "Running Near Field Edge visualization test..."
        echo "You should see robot navigating near field boundaries with obstacles."
        ./test_static_obstacles_integration --gtest_filter=*VisualizeNearFieldEdge*
        ;;
    "all")
        echo "Running all visualization tests..."
        ./test_static_obstacles_integration --gtest_filter=*Visualize*
        ;;
    *)
        echo "Unknown test: $1"
        echo "Available tests: wall, moving, edge, all"
        exit 1
        ;;
esac
