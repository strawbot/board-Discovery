#!/usr/bin/env python3
"""
update_aliases.py
─────────────────
Reads pin definitions from a STM32CubeMX-generated main.h and patches the
alias_table[] block inside boards/gpio_dump.c (or any path you specify).

STM32CubeMX writes pairs like:
    #define SPI1_MISO_Pin        GPIO_PIN_6
    #define SPI1_MISO_GPIO_Port  GPIOA

This script matches those pairs, converts them to port_index / pin numbers,
and rewrites the alias_table[] content in gpio_dump.c.

Usage:
    python update_aliases.py                          # uses default paths below
    python update_aliases.py main.h boards/gpio_dump.c
"""

import re
import sys
from pathlib import Path
from typing import Dict, List, Optional

# ─── Default file paths (edit here or pass as arguments) ──────────────────────
DEFAULT_MAIN_H    = "main.h"
DEFAULT_GPIO_DUMP = "boards/gpio_dump.c"

# ─── Port letter → port_table[] index mapping ─────────────────────────────────
# Must match the order of port_table[] in gpio_dump.c:
#   { GPIOA,"PA" }, { GPIOB,"PB" }, { GPIOC,"PC" }, ...
PORT_INDEX = {
    "GPIOA": 0, "GPIOB": 1, "GPIOC": 2, "GPIOD": 3,
    "GPIOE": 4, "GPIOF": 5, "GPIOG": 6, "GPIOH": 7,
    "GPIOI": 8, "GPIOJ": 9, "GPIOK": 10,
}

# ─── GPIO_PIN_N → bit-position (pin number) ───────────────────────────────────
GPIO_PIN_NAME = {f"GPIO_PIN_{n}": n for n in range(16)}


def _resolve_pin_value(raw: str) -> Optional[int]:
    """
    Convert a raw pin value token to a 0-15 pin number.

    Handles the two formats CubeMX generates:
      Named macro : GPIO_PIN_6           → 6
      Hex bitmask : 0x0040U / 0x40       → 6   (bit_length - 1)
      Decimal     : 64                   → 6   (bit_length - 1)
    Returns None if the value cannot be mapped to a single valid pin.
    """
    raw = raw.strip()

    # ── Named macro (HAL: GPIO_PIN_N  or  LL: LL_GPIO_PIN_N) ────────────────
    if raw.startswith("LL_GPIO_PIN_"):
        raw = raw[len("LL_"):]                  # strip prefix → GPIO_PIN_N
    if raw.startswith("GPIO_PIN_"):
        return GPIO_PIN_NAME.get(raw)           # None if not 0-15

    # ── Numeric literal (hex or decimal, optional U/u suffix) ────────────────
    numeric = raw.rstrip("UuLl")                # strip type suffixes
    try:
        value = int(numeric, 0)                 # int("0x0040", 0) → 64
    except ValueError:
        return None

    if value == 0 or (value & (value - 1)) != 0:
        return None                             # not a power-of-two → invalid

    pin = value.bit_length() - 1
    return pin if 0 <= pin <= 15 else None


# ──────────────────────────────────────────────────────────────────────────────
def diagnose_main_h(path: Path) -> None:
    """
    Print every #define line that contains 'Pin' or 'Port' so the user can
    see exactly what format the file uses.  Run with --diagnose.
    """
    text = path.read_text(encoding="utf-8")
    print(f"=== Lines in '{path}' that mention Pin or Port ===\n")
    for i, line in enumerate(text.splitlines(), 1):
        if re.search(r"#define\s+\w+_(Pin|GPIO_Port)\b", line):
            print(f"  {i:4d}: {line.rstrip()}")
    print()


# ──────────────────────────────────────────────────────────────────────────────
def parse_main_h(path: Path) -> List[Dict]:
    """
    Parse main.h and return a list of dicts:
        { "name": "SPI1_MISO", "port": "GPIOA", "pin": 6, "port_idx": 0 }

    Accepts both CubeMX pin-value formats:
        #define SPI1_MISO_Pin        GPIO_PIN_6    ← named macro
        #define SPI1_MISO_Pin        0x0040U       ← hex bitmask
    """
    # Broad pin regex — captures whatever token follows _Pin
    pin_re  = re.compile(r"^\s*#define\s+(\w+)_Pin\s+(\S+)", re.M)
    port_re = re.compile(r"^\s*#define\s+(\w+)_GPIO_Port\s+(GPIO\w+)", re.M)

    text = path.read_text(encoding="utf-8")

    raw_pins = {m.group(1): m.group(2) for m in pin_re.finditer(text)}
    ports    = {m.group(1): m.group(2) for m in port_re.finditer(text)}

    results = []
    warned  = False

    for name in sorted(raw_pins):
        raw_val   = raw_pins[name]              # e.g. "GPIO_PIN_6" or "0x0040U"
        port_name = ports.get(name)             # e.g. "GPIOA"

        if port_name is None:
            print(f"  [WARN] {name}_Pin found but no matching {name}_GPIO_Port — skipped")
            warned = True
            continue

        pin_num = _resolve_pin_value(raw_val)
        if pin_num is None:
            print(f"  [WARN] Cannot decode pin value '{raw_val}' for {name} — skipped")
            print(f"         (run with --diagnose to inspect the file)")
            warned = True
            continue

        port_idx = PORT_INDEX.get(port_name)
        if port_idx is None:
            print(f"  [WARN] Unknown port '{port_name}' for {name} — skipped")
            warned = True
            continue

        results.append({
            "name":     name,
            "port":     port_name,
            "port_idx": port_idx,
            "pin":      pin_num,
        })

    if not results:
        print("  [WARN] No pin definitions found.")
        print("         Run with --diagnose to see what the file actually contains.")
    elif warned:
        print()

    return results


# ──────────────────────────────────────────────────────────────────────────────
def build_alias_block(pins: List[Dict]) -> str:
    """
    Build the replacement content for the alias_table[] initialiser list.
    Pins are grouped by port for readability.
    """
    if not pins:
        return "    /* No pins found in main.h */\n"

    lines = []
    current_port = None

    # Sort by port index, then pin number within each port
    for p in sorted(pins, key=lambda x: (x["port_idx"], x["pin"])):
        port_label = f"port {p['port_idx']} = {p['port']}"

        if p["port"] != current_port:
            if current_port is not None:
                lines.append("")            # blank line between ports
            # Port comment header ────────────────────────────────────────────
            dashes = "─" * (35 - len(port_label))
            lines.append(f"    /* ── {port_label} {dashes} */")
            current_port = p["port"]

        # Pad the alias name so the port column lines up
        name_padded = f'"{p["name"]}"'.ljust(20)

        # Build pin prefix label e.g. "P" + "A" + "6"
        port_letter = p["port"].replace("GPIO", "P")   # GPIOA → PA
        pin_label   = f"{port_letter}{p['pin']}"        # e.g. PA6

        lines.append(
            f'    {{ {p["port_idx"]}, {p["pin"]:>2}, {name_padded} }},   '
            f'/* {pin_label:<5} */'
        )

    return "\n".join(lines) + "\n"


# ──────────────────────────────────────────────────────────────────────────────
def patch_gpio_dump(c_path: Path, new_block: str) -> int:
    """
    Replace the body of alias_table[] in gpio_dump.c with new_block.
    Returns the number of entries written, or -1 on failure.

    Matches the region between the opening '{' of alias_table[] and the
    closing '};'  (handles any spacing / comments around them).
    """
    text = c_path.read_text(encoding="utf-8")

    # Pattern: static const PinAlias_t alias_table[] = {  …contents…  };
    pattern = re.compile(
        r"(static\s+const\s+PinAlias_t\s+alias_table\[\]\s*=\s*\{)"  # group 1: header
        r".*?"                                                          # old body (non-greedy)
        r"(\s*\};)",                                                    # group 2: closing
        re.DOTALL
    )

    match = pattern.search(text)
    if not match:
        print("  [ERROR] Could not locate alias_table[] in the .c file.")
        print("          Make sure the declaration matches:")
        print("          static const PinAlias_t alias_table[] = { ... };")
        return -1

    replacement = f"{match.group(1)}\n{new_block}{match.group(2)}"
    new_text = text[: match.start()] + replacement + text[match.end() :]

    # Write with a .bak backup
    backup = c_path.with_suffix(".c.bak")
    backup.write_text(text, encoding="utf-8")

    c_path.write_text(new_text, encoding="utf-8")
    return new_block.count("{")      # rough count = number of entries


# ──────────────────────────────────────────────────────────────────────────────
def main():
    args = sys.argv[1:]

    # ── --diagnose flag: print raw Pin/Port lines and exit ───────────────────
    if "--diagnose" in args:
        args.remove("--diagnose")
        main_h = Path(args[0] if args else DEFAULT_MAIN_H)
        if not main_h.exists():
            print(f"[ERROR] '{main_h}' not found.")
            sys.exit(1)
        diagnose_main_h(main_h)
        sys.exit(0)

    main_h = Path(args[0] if len(args) > 0 else DEFAULT_MAIN_H)
    dump_c = Path(args[1] if len(args) > 1 else DEFAULT_GPIO_DUMP)

    print(f"Parsing  : {main_h}")
    print(f"Patching : {dump_c}")
    print()

    # ── Validate inputs ──────────────────────────────────────────────────────
    if not main_h.exists():
        print(f"[ERROR] '{main_h}' not found.")
        sys.exit(1)
    if not dump_c.exists():
        print(f"[ERROR] '{dump_c}' not found.")
        sys.exit(1)

    # ── Parse ────────────────────────────────────────────────────────────────
    pins = parse_main_h(main_h)
    print(f"Found {len(pins)} pin definition(s):")
    for p in sorted(pins, key=lambda x: (x["port_idx"], x["pin"])):
        port_letter = p["port"].replace("GPIO", "P")
        print(f"  {port_letter}{p['pin']:<3}  {p['name']}")
    print()

    # ── Build replacement block ───────────────────────────────────────────────
    new_block = build_alias_block(pins)

    # ── Patch gpio_dump.c ────────────────────────────────────────────────────
    count = patch_gpio_dump(dump_c, new_block)
    if count >= 0:
        backup = dump_c.with_suffix(".c.bak")
        print(f"Done. {count} alias entries written to '{dump_c}'.")
        print(f"Original backed up as '{backup}'.")
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()