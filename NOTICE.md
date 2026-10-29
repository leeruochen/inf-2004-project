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

## Course project template

INF2004 course project template provided by the module instructor.

- Author: TODO(owner-to-confirm)
- Kept unchanged in `ProjectTemplate/` as reference only. Its layout
  (`app_program/subsystemN/`, `TEAM.md`, `testing/`, `videos/`) and the
  patterns listed in `docs/TEMPLATE_NOTES.md` are adapted in this repo.
- The template carries no licence file of its own (UNVERIFIED). Its kernel
  files keep their T-License 2.2 headers.

## Raspberry Pi pico-examples (planned)

The wheel encoders will use `pio/quadrature_encoder/quadrature_encoder.pio`
from Raspberry Pi pico-examples, tag `sdk-1.5.1`
(https://github.com/raspberrypi/pico-examples/tree/sdk-1.5.1/pio/quadrature_encoder).
It is copied in Batch B (it is not in the repo yet) and must keep its
original header: "Copyright (c) 2023 Raspberry Pi (Trading) Ltd.",
`SPDX-License-Identifier: BSD-3-Clause`. The pico-examples repository licence
(`LICENSE.TXT` at that tag):

```
Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
   disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
