lacam2
---
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

## Building

```sh
git clone --recursive https://github.com/Kei18/lacam2.git
cd lacam2
cmake -B build && make -C build
```


## Notes

- Auto formatting (clang-format) when committing:

```sh
git config core.hooksPath .githooks && chmod a+x .githooks/pre-commit
```
