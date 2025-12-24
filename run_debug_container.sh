docker run -it --rm \
  -v $(pwd):/workspace \
  -v $(realpath data):/data \
  mf-eye-runtime:humble
