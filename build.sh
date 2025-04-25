#!/bin/bash

set -e  # Exit on any error

echo "Compiling planner.cpp..."
g++ planner.cpp -o planner.out
echo "✔ planner.out built"
chmod +x planner.out

echo "Compiling verifier.cpp..."
g++ verifier.cpp -o verifier.out
echo "✔ verifier.out built"
chmod +x verifier.out

echo "Compiling config_checker.cpp..."
g++ config_checker.cpp -o config_checker.out
echo "✔ config_checker.out built"
chmod +x config_checker.out

echo "✅ All files compiled successfully."

