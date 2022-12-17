lacam2
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)
[![CI](https://github.com/Kei18/lacam2/actions/workflows/ci.yml/badge.svg)](https://github.com/Kei18/lacam2/actions/workflows/ci.yml)

The code repository of the paper "Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding."

## Building

All you need is [CMake](https://cmake.org/) (≥v3.16). The code is written in C++(17).

First, clone this repo with submodules.

```sh
git clone --recursive https://github.com/Kei18/lacam2.git && cd lacam2
```

Then, build the project.

```sh
cmake -B build && make -C build
```

### for M1 cpu
```sh
cmake -B build -DCPU=M1 && make -C build
```

### Docker

You can also use the [docker](https://www.docker.com/) environment (based on Ubuntu18.04) instead of the native one.

```sh
# ~10 min, mostly for CMake build
docker compose up -d
docker compose exec dev bash
> cmake -B build && make -C build
```

## Usage

makespan optimization:

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 2 --objective 1
solved: 8ms     makespan: 10 (lb=2, ub=5)       sum_of_costs: 21 (lb=5, ub=4.2) sum_of_loss: 21 (lb=5, ub=4.2)
```

sum-of-loss optimization:

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 2 --objective 2
solved: 0ms     makespan: 11 (lb=2, ub=5.5)     sum_of_costs: 23 (lb=5, ub=4.6) sum_of_loss: 19 (lb=5, ub=3.8)
```

You can find details of all parameters with:
```sh
build/main --help
```

## Visualizer

This repository is compatible with [@Kei18/mapf-visualizer](https://github.com/kei18/mapf-visualizer).

## Experiments

The experimental script is written in Julia ≥1.7.
Setup may require around 10 minutes.

```sh
sh scripts/setup.sh
```

Edit the config file as you like.
Examples are in `scripts/config` .
The evaluation starts by following commands.

```
julia --project=scripts/ --threads=auto
> include("scripts/eval.jl"); main("scripts/config/mapf-bench.yaml")
```

## Notes

- The grid maps and scenarios in `assets/` are from [MAPF benchmarks](https://movingai.com/benchmarks/mapf.html).
- `tests/` is not comprehensive. It was used in early developments.
- Auto formatting (clang-format) when committing:

```sh
git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
```

## Licence

This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author

[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at Tokyo Institute of Technology, interested in controlling multiple moving agents.
