"""testing/subsystem4/fw_build.py - build robocar.uf2 without the hanging link.

On some Windows PCs the final link step of `cmake --build` (ninja starting
arm-none-eabi-g++ through `cmd.exe /C`) never finishes: g++ sits idle and the
build waits forever (19 Nov 2026, Buddy 4's PC; running the same command by
hand works).  This script:
  1. lets ninja compile every object file (no link, so nothing hangs),
  2. writes the linker response file (CMakeFiles/robocar.rsp) from build.ninja,
  3. runs the link, objcopy and elf2uf2 itself, each with a time limit.
Every step has a timeout, so a stuck tool stops the build with an error
instead of hanging.

Usage (from the repo root, after `cmake -S . -B <dir> ...` has configured it):
    python testing/subsystem4/fw_build.py build/s4hump
Called by s4_test.ps1.  Uses only the Python standard library.
"""

import os
import re
import subprocess
import sys

COMPILE_TIMEOUT_S = 900
LINK_TIMEOUT_S = 240


def run(cmd, timeout_s, cwd, shell):
    """Run one command with stdin closed; return its exit code (124 = timeout)."""
    try:
        res = subprocess.run(cmd, cwd=cwd, shell=shell, stdin=subprocess.DEVNULL,
                             timeout=timeout_s)
        return res.returncode
    except subprocess.TimeoutExpired:
        print(f"fw_build: TIMEOUT after {timeout_s} s: {str(cmd)[:120]}")
        return 124


def link_edge(build_ninja):
    """Inputs and variables of the `build robocar.elf:` edge."""
    text = open(build_ninja, encoding="utf-8").read()
    start = text.index("\nbuild robocar.elf:") + 1
    line = ""
    pos = start
    while True:
        end = text.index("\n", pos)
        seg = text[pos:end]
        if seg.endswith("$"):
            line += seg[:-1]
            pos = end + 1
            continue
        line += seg
        break
    deps = line.split(":", 1)[1].split("|")[0].split()[1:]
    block = text[end + 1:text.index("\n\n", end + 1)]
    var = dict(re.findall(r"^  (\w+) = (.*)$", block, re.M))
    return deps, var


def link_commands(rules_ninja, var):
    """The link rule's command, split at '&&', without the slow/unused parts."""
    rules = open(rules_ninja, encoding="utf-8").read()
    k = rules.index("rule CXX_EXECUTABLE_LINKER__robocar_")
    cmd = re.search(r"command = (.*)", rules[k:]).group(1)
    for name, val in var.items():
        cmd = cmd.replace("$" + name, val)
    cmd = cmd.replace("$out", "robocar.elf").replace("$$", "$")
    prefix = 'cmd.exe /C "'
    if cmd.startswith(prefix) and cmd.endswith('"'):
        cmd = cmd[len(prefix):-1]
    parts = [p.replace(prefix, "").rstrip('"').strip() for p in cmd.split(" && ")]
    keep = []
    for p in parts:
        if p in ("cd .", "") or p.startswith("cd /D") or "objdump" in p or "-Oihex" in p:
            continue    # disassembly and .hex are not needed to flash
        keep.append(p)
    return keep


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    bdir = os.path.abspath(sys.argv[1])
    ninja = os.path.join(os.environ.get("PICO_SDK_ROOT", r"C:\Program Files\Raspberry Pi\Pico SDK v1.5.1"),
                         "ninja", "ninja.exe")

    deps, var = link_edge(os.path.join(bdir, "build.ninja"))
    targets = [d for d in deps if d.endswith(".obj") or d.endswith(".a")]

    print(f"fw_build: compiling {len(targets)} inputs with ninja (no link)")
    if run([ninja, "-C", bdir] + targets, COMPILE_TIMEOUT_S, None, False) != 0:
        print("fw_build: compile FAILED (scroll up for the first error)")
        return 1

    rsp = os.path.join(bdir, "CMakeFiles", "robocar.rsp")
    with open(rsp, "w", encoding="utf-8") as f:
        f.write(" ".join(deps) + " " + var.get("LINK_PATH", "") + " " + var["LINK_LIBRARIES"])

    for cmd in link_commands(os.path.join(bdir, "CMakeFiles", "rules.ninja"), var):
        print(f"fw_build: {cmd[:90]}...")
        if run(cmd, LINK_TIMEOUT_S, bdir, True) != 0:
            print("fw_build: link step FAILED")
            return 1

    uf2 = os.path.join(bdir, "robocar.uf2")
    if not os.path.exists(uf2):
        print("fw_build: robocar.uf2 was not produced")
        return 1
    print(f"fw_build: OK {uf2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
