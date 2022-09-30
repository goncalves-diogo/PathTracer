#/bin/sh

cmake -DGLFW_BUILD_DOCS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -S . -B build/

cd build/
make
cd bin/
# Makefile

./ESCViewer2021 -m models/cornell/CornellBox-Mirror.obj

open output.ppm
