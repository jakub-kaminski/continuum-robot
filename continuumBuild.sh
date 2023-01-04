#!/bin/sh
# Script for transcompiling to C++ to .WASM using the Emscripten Toolchain

js_dir="/home/jakub/Documents/robotsim-org/robot-simulator-web"

rm -f ${js_dir}/public/moduleContinuum.wasm
rm -rf ${js_dir}/public/moduleContinuum.worker.js
rm -f ${js_dir}/src/simulations/ContinuumRobot/moduleContinuum.mjs

#rm -rf $(pwd)/build-emscripten
#mkdir -p $(pwd)/build-emscripten

emcmake cmake -B $(pwd)/build-emscripten -S $(pwd) -DCMAKE_BUILD_TYPE=Release
cmake --build $(pwd)/build-emscripten --target moduleContinuum -j 8

mv $(pwd)/build-emscripten/wasm/moduleContinuum.wasm ${js_dir}/public/moduleContinuum.wasm
mv $(pwd)/build-emscripten/wasm/moduleContinuum.mjs ${js_dir}/src/simulations/ContinuumRobot/moduleContinuum.mjs
