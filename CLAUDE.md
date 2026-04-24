# Discovery/ — Binding rules for Claude

This folder is the Discovery board project (STM32F407, STM32CubeIDE, CMake).
Authoritative project-wide rules live at the top-level `CLAUDE.md` and at
`Robot/CLAUDE.md`. Read those first; the rules below are the Discovery-
specific reinforcements.

## Sibling-ignorance

Discovery must not reach into any other board folder. Do not include from,
reference paths inside, or copy-paste out of `Nano/`, `Nucleo411/`,
`Nucleo446/`, `PNucleo/`, or `TIVA/`. If another board has a feature
Discovery also wants, factor it into `Robot/` and compose it here — never
cross-link.

## Compose from Robot, don't duplicate it

Shared features live in `Robot/`. Discovery adopts them by:

1. Adding `../Robot/` as a linked source folder in STM32CubeIDE (one-time),
   and adding the relevant Robot sources/includes to Discovery's
   `CMakeLists.txt`.
2. Calling `feature_init()` (and any hook registrations) from Discovery's
   startup path.
3. Supplying hardware shims the feature requests via its `feature_bind_hw()`
   API — shims live **here**, in Discovery.

## lwIP — the split

Keep only the Discovery-specific port in `Discovery/LWIP/`:
`lwipopts.h`, `sys_arch.*`, `cc.h`, `ethernetif.*`, and the PHY/MAC glue.
The protocol core (`src/core/`, `src/api/`, generic `src/netif/`) must not
be duplicated here — it lives once under `Robot/net/lwip/` and is pulled in
through Discovery's CMake. If you see a divergent copy of the lwIP core
under `Discovery/`, the fix is to delete it and point the build at Robot.

## What belongs in Discovery/

- `Core/`, `Drivers/`, `Middlewares/` (less the lwIP core), STM32CubeIDE
  artifacts (`.ioc`, `.launch`, linker scripts, `startup_*.s`).
- The Discovery lwIP port (`Discovery/LWIP/`) and any PHY-specific driver.
- Board-specific CLI words (anything that reads STM32 registers, dumps HAL
  state, etc.). Register them with `cli_register(...)` from a Discovery
  init function.
- The Discovery `main` that wires Robot features together.

## What does NOT belong in Discovery/

- Board-agnostic protocol code or diagnostics. Those belong in `Robot/`.
- A private copy of the lwIP protocol core.
- Anything another board would need to copy to use.

## When adding a feature

If it could run on any other board with hardware glue swapped out, factor
it into `Robot/` first, then adopt it here. Do not implement as a
Discovery-local module that later has to be generalized.
