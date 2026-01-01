#!/bin/bash
### share the output to chatgpt to create DBML script
### and open https://dbdiagram.io/d to see results
set -e

OUT="_schema_all.sql"

# 1. remove existing output file
if [ -f "${OUT}" ]; then
  rm "${OUT}"
fi

# 2. merge all sql files in current directory
for f in *.sql; do
  if [ "$f" != "${OUT}" ]; then
    cat "$f" >> "${OUT}"
    echo >> "${OUT}"
  fi
done

echo "generated ${OUT}"

