cmake -H. -B_build/Release -DCMAKE_BUILD_TYPE=Release && cmake --build _build/Release -- -j8
echo -e "\n\033[1;33m Workspace build complete \033[0m\n"

if ! [[ $? -eq 0 ]]; then
    echo FAIL
    exit 1
fi

export PATH=/workspace/_bin:${PATH}

if ! [[ -d /workspace/data ]]; then
    mkdir -p /workspace/data
fi

"$@"

if ! [[ $? -eq 0 ]]; then
    echo FAIL
    exit 1
fi

echo -e "\n\033[1;33m Working on visualisation. This may take some time ... \033[0m\n"
python3.6 scripts/plot.py

filename=data_$(date +%Y-%m-%d--%H%M%S).tar.gz
tar czf ${filename} data
rm -rf data
echo -e "\n\033[1;33m Data saved to ${filename} \033[0m\n"

if ! [[ $? -eq 0 ]]; then
    echo FAIL
    exit 1
fi

exit 0
