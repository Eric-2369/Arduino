#!/bin/bash

# This script formats all .ino, .cpp, and .h files in the src directory using clang-format.

# Check if clang-format is installed
if ! command -v clang-format &> /dev/null; then
    echo "clang-format could not be found. Please install it first."
    exit 1
fi

# Define the source directory
SRC_DIR="./src"

# Check if the source directory exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Source directory $SRC_DIR does not exist."
    exit 1
fi

# Find and format all .ino, .cpp, and .h files in the src directory
echo "Formatting all .ino, .cpp, and .h files in the src directory..."

find "$SRC_DIR" -type f \( -name '*.ino' -o -name '*.cpp' -o -name '*.h' \) | while read -r file; do
    echo "Formatting $file"
    clang-format -i "$file"
done

echo "Formatting completed."
