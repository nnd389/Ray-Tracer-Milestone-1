#!/bin/sh
set -e

NAME="$1"
if [ -z "$NAME" ]; then
  echo "usage: $0 <archive_base_name>"
  exit 1
fi

STAGE="ray"
rm -rf "$STAGE"
mkdir -p "$STAGE"

cp -r src "$STAGE/"
cp -r cmake "$STAGE/"
cp CMakeLists.txt "$STAGE/"
cp README.md "$STAGE/"

# Remove macOS metadata files if any slipped in
find "$STAGE" -name "._*" -delete

tar -czf "${NAME}.tgz" "$STAGE"
rm -rf "$STAGE"

echo "Created ${NAME}.tgz"
