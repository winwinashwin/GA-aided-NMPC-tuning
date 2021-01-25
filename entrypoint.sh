#!/bin/bash

BUILD_CONFIGURATION="Release"
DATA_VIS_SCRIPT="plot.py"
DATA_RAW_DIR="data"

function verifyOK () {
    if [[ $? -ne 0 ]]; then
        echo FAIL
        exit 1
    fi
}

function consoleLog () {
    echo -e "\n\033[1;33m $1 \033[0m\n"
}

function preBuild () {
    if [[ -d build ]]; then
        echo
        read -p "? Found build directory. Delete and build from scratch? (y/n): " confirm
        [[ $confirm == [yY] ]] && rm -rf build
    fi
}

function buildProject () {
    cmake -H. -Bbuild/${BUILD_CONFIGURATION} -DCMAKE_BUILD_TYPE=${BUILD_CONFIGURATION} && \
    cmake --build build/${BUILD_CONFIGURATION} -- -j8
}

function postBuild () {
    export LD_LIBRARY_PATH=`pwd`/build/${BUILD_CONFIGURATION}/lib:$LD_LIBRARY_PATH
    export PATH=`pwd`/build/${BUILD_CONFIGURATION}/bin:$PATH
}

function plotData () {
    python3 ${DATA_VIS_SCRIPT}
}

function saveData () {
    filename=data_$(date +%Y-%m-%d--%H%M%S).zip
    zip -rq ${filename} data
    rm -rf data
}

preBuild
buildProject
postBuild
verifyOK
consoleLog "Workspace build complete"

"$@"
verifyOK

consoleLog "Working on visualisation. This may take some time ..."
plotData
saveData
verifyOK
consoleLog "Data saved to ${filename}"

exit 0
