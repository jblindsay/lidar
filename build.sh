#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ "$DIR" != pwd ]; then
  cd $DIR
fi

echo "Crystal is formating the source code..."
crystal tool format

echo "Running Crystal doc..."
crystal doc

# echo "Unit tests..."
# crystal spec

echo "Crystal is building Lidar..."
crystal build src/lidar.cr --release

echo "Running code..."
time ./lidar
