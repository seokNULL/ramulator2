  #!/bin/bash
  set -e

  mkdir -p build_rel
  cd build_rel
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j
  cp ./ramulator2 ../ramulator2
  cd ..