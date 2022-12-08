#/bin/sh

cmake -DGLFW_BUILD_DOCS=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -S . -B build/

cd build/
make
cd bin/
# Makefile

./Tracer -m models/cornell/CornellBox-Empty-CO.obj

if [[ "$OSTYPE" == "darwin"* ]]; then
    open output.ppm
else
    feh output.ppm
fi
