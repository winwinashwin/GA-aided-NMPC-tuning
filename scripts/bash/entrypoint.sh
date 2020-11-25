#!/bin/bash

BUILD_CONFIGURATION="Release"
DATA_VIS_SCRIPT="scripts/python/plot.py"
DATA_RAW_DIR="data"

function verifyOK () {
    if ! [[ $? -eq 0 ]]; then
        echo FAIL
        exit 1
    fi
}

function consoleLog () {
    echo -e "\n\033[1;33m $1 \033[0m\n"
}

cmake -H. -Bbuild/${BUILD_CONFIGURATION} -DCMAKE_BUILD_TYPE=${BUILD_CONFIGURATION} && \
cmake --build build/${BUILD_CONFIGURATION} -- -j8
verifyOK
consoleLog "Workspace build complete"

export PATH=/workspace/bin:${PATH}

"$@"

verifyOK

consoleLog "Working on visualisation. This may take some time ..."
python3.6 ${DATA_VIS_SCRIPT}

filename=data_$(date +%Y-%m-%d--%H%M%S).zip
zip -rq ${filename} data
rm -rf data
consoleLog "Data saved to ${filename}"

verifyOK

exit 0
