# Notice

owner: 16

This product uses the source code of micro T-Kernel 3.0 BSP (uT-Kernel 3.0
BSP, branch `pico_rp2040`) released by TRON Forum (https://www.tron.org),
under the T-License 2.2.

- Source: https://github.com/tron-forum/mtk3_bsp (vendored as the git
  submodule `external/mtk3_bsp`, pinned to commit
  `15ed232c08f2d89e54e514db79de150e224307e3`).
- Licence text: [external/mtk3_bsp/docs/TEF000-219-200401.pdf](external/mtk3_bsp/docs/TEF000-219-200401.pdf)
  ([online](https://github.com/tron-forum/mtk3_bsp/blob/pico_rp2040/docs/TEF000-219-200401.pdf)).
- BSP manual: [external/mtk3_bsp/docs/uTK3bsp_pico_man_jp.pdf](external/mtk3_bsp/docs/uTK3bsp_pico_man_jp.pdf).
- Every vendored source keeps its original copyright and licence header. The
  project copies of the kernel configuration in `config/mtk3/` keep theirs too.

`cmake/memmap_mtk3.ld` and `pico_sdk_import.cmake` are copied from the
Raspberry Pi Pico SDK 1.5.1, Copyright (c) 2020 Raspberry Pi (Trading) Ltd.,
BSD-3-Clause.
