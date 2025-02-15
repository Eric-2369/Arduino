#!/bin/bash

# List of source files to be linked
SOURCE_FILES=(
    "lib/deviceHandlers/deviceHandlers.h"
    "lib/deviceHandlers/deviceHandlers.cpp"
    # Add more library files here
)

# List of target directories, add or remove directories as needed
TARGET_DIRS=(
    "src/climate-monitor-shanghai"
    "src/climate-monitor-wuhan"
    "src/climate-monitor-wuhan-office"
    "src/sensor-fusion-display"
    "src/spectrometer-display"
    # Add more target directories here
)

# Function to create symbolic links
create_symlink() {
    local target_dir=$1
    local source_file=$2
    local filename=$(basename "$source_file")

    # Calculate the relative path to the source file from the target directory
    local relative_source_path=$(realpath --relative-to="$target_dir" "$source_file")

    # Check if the file or symlink already exists
    if [ -e "$target_dir/$filename" ] || [ -L "$target_dir/$filename" ]; then
        echo "Removing existing file or symlink: $target_dir/$filename"
        rm -f "$target_dir/$filename"
    fi

    echo "Creating symbolic link: $target_dir/$filename -> $relative_source_path"
    ln -s "$relative_source_path" "$target_dir/$filename"
}

# Iterate over target directories and create symbolic links for each source file
for target_dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$target_dir" ]; then
        for source_file in "${SOURCE_FILES[@]}"; do
            create_symlink "$target_dir" "$source_file"
        done
    else
        echo "Target directory does not exist: $target_dir"
    fi
done

echo "Symbolic link creation completed."
