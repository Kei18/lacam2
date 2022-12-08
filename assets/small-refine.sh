#!/bin/sh

function getDate() {
    if [ "$(uname)" == "Darwin" ]; then
        gdate +%Y-%m-%d_%H-%M-%S-%2N
    else  # linux
        date +%Y-%m-%d_%H-%M-%S-%2N
    fi
}

instances=(1 2 3 4 5)
time_limit_sec=30

OUTPUT_DIR=../data/exp/`getDate`
mkdir -p $OUTPUT_DIR

map_name=tunnel
N=4
for k in ${instances[@]}
do
    build/main \
        -t ${time_limit_sec} \
        -O 2 \
        -m scripts/map/${map_name}.map \
        -N $N \
        -i assets/${map_name}${k}.scen \
        -o $OUTPUT_DIR/${map_name}${k}.txt
done

map_name=loop-chain
N=5
for k in ${instances[@]}
do
    build/main \
        -t ${time_limit_sec} \
        -O 2 \
        -m scripts/map/${map_name}.map \
        -N $N \
        -i assets/${map_name}${k}.scen \
        -o $OUTPUT_DIR/${map_name}${k}.txt
done

map_name=random-32-32-20
N=30
for k in ${instances[@]}
do
    build/main \
        -t ${time_limit_sec} \
        -O 2 \
        -m scripts/map/${map_name}.map \
        -N $N \
        -i scripts/scen/scen-random/${map_name}-random-${k}.scen \
        -o $OUTPUT_DIR/${map_name}-${N}-${k}.txt \
        -l
done

map_name=random-32-32-20
N=100
for k in ${instances[@]}
do
    build/main \
        -t ${time_limit_sec} \
        -O 2 \
        -m scripts/map/${map_name}.map \
        -N $N \
        -i scripts/scen/scen-random/${map_name}-random-${k}.scen \
        -o $OUTPUT_DIR/${map_name}-${N}-${k}.txt \
        -l
done
