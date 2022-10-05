lacam2
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

## Building

```sh
git clone --recursive https://github.com/Kei18/lacam2.git
cd lacam2
cmake -B build && make -C build
```

## Usage

```sh
> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 2 --objective 1
solved: 8ms     makespan: 10 (lb=2, ub=5)       sum_of_costs: 21 (lb=5, ub=4.2) sum_of_loss: 21 (lb=5, ub=4.2)

> build/main -m assets/loop.map -i assets/loop.scen -N 3 -v 2 --objective 2
solved: 0ms     makespan: 11 (lb=2, ub=5.5)     sum_of_costs: 23 (lb=5, ub=4.6) sum_of_loss: 19 (lb=5, ub=3.8)
```

## Notes

- Auto formatting (clang-format) when committing:

```sh
git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
```
