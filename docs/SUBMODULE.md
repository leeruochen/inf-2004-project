# Kernel submodule (external/mtk3_bsp)

`external/mtk3_bsp` is the TRON Forum micro T-Kernel 3.0 BSP
(https://github.com/tron-forum/mtk3_bsp.git, branch `pico_rp2040`), distributed
under the T-License 2.2. The licence text is
`external/mtk3_bsp/docs/TEF000-219-200401.pdf`. The BSP manual (Japanese) is
`external/mtk3_bsp/docs/uTK3bsp_pico_man_jp.pdf`.

## Pin

Pinned to `15ed232c08f2d89e54e514db79de150e224307e3` (2023-05-24, "Update manual").

It was added with:

```
git submodule add -b pico_rp2040 https://github.com/tron-forum/mtk3_bsp.git external/mtk3_bsp
git -C external/mtk3_bsp checkout --detach 15ed232c08f2d89e54e514db79de150e224307e3
git add .gitmodules external/mtk3_bsp
```

## Getting it

Fresh clone:

```
git clone --recurse-submodules https://github.com/leeruochen/inf-2004-project.git
```

Existing clone:

```
git submodule update --init --recursive
```

## Verifying the hash

```
git submodule status
git -C external/mtk3_bsp rev-parse HEAD
```

`git submodule status` must print a line starting with a space (not `+` or `-`)
followed by `15ed232c08f2d89e54e514db79de150e224307e3 external/mtk3_bsp`.

CMake configure checks this too: `cmake/mtk3.cmake` fails if the submodule is
off the pin or has local changes. `-DMTK3_ALLOW_UNPINNED=ON` turns that into a
warning, for experiments only.

## Rules

- Never run `git submodule update --remote`. It moves off the pin.
- Never edit vendored files. Kernel configuration lives in `config/mtk3/`
  (project copies, changes marked `PROJECT:`).
- Bumping the pin needs owner approval. Then re-run the kernel self-test
  (Batch B) and update the hash in `cmake/mtk3.cmake` and in this file.
