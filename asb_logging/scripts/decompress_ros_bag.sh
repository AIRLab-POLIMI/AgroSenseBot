#!/bin/bash

# Input directory (default: current dir)
INPUT_DIR="${1:-.}"
INPUT_DIR="$(realpath "$INPUT_DIR")"

# Threads per zstd process (optional second arg), default: all CPU cores
ZSTD_THREADS="${2:-$(nproc)}"

# Max concurrent jobs (optional third arg), default: 4
MAX_PARALLEL="${3:-4}"

# Output directory
BASE_NAME="$(basename "$INPUT_DIR")"
OUTPUT_DIR="$(dirname "$INPUT_DIR")/${BASE_NAME}_decompressed"
mkdir -p "$OUTPUT_DIR"

echo "Using $ZSTD_THREADS threads per file, max $MAX_PARALLEL parallel jobs."
echo "Output directory: $OUTPUT_DIR"

export INPUT_DIR OUTPUT_DIR ZSTD_THREADS

handle_file() {
    FILE="$1"
    REL_PATH="${FILE#$INPUT_DIR/}"

    # Skip top-level metadata.yaml
    if [[ "$REL_PATH" == "metadata.yaml" ]]; then
        echo "* Skipping:        $REL_PATH"
        return
    fi

    EXT="${FILE##*.}"

    if [[ "$EXT" == "zst" || "$EXT" == "zstd" ]]; then
        RAW_FILE="${REL_PATH%.zst}"
        RAW_FILE="${RAW_FILE%.zstd}"
        RAW_PATH="$INPUT_DIR/$RAW_FILE"
        OUT_PATH="$OUTPUT_DIR/$RAW_FILE"
        mkdir -p "$(dirname "$OUT_PATH")"

        if [[ -f "$RAW_PATH" ]]; then
            echo "* Skipping:        $REL_PATH (original $RAW_FILE exists)"
        else
            echo "* Decompressing:   $REL_PATH"
            zstd -d -c --threads="$ZSTD_THREADS" "$FILE" > "$OUT_PATH"
        fi
    else
        OUT_PATH="$OUTPUT_DIR/$REL_PATH"
        mkdir -p "$(dirname "$OUT_PATH")"
        echo "* Copying:         $REL_PATH"
        cp "$FILE" "$OUT_PATH"
    fi
}

export -f handle_file

# Find and process all files in parallel
find "$INPUT_DIR" -type f -print0 | \
    xargs -0 -n1 -P "$MAX_PARALLEL" bash -c 'handle_file "$0"'

# Run reindex
echo "Running ros2 bag reindex on $OUTPUT_DIR ..."
ros2 bag reindex "$OUTPUT_DIR"

echo "Done"
