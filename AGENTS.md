# Project Memory

## Build Commands

This CCS/TI Clang project generates its makefile under `Debug/`, not the
repository root. Run `gmake` commands from:

```powershell
D:\TiCompetition\2210B\AutoParking_MSPM0G3507_nortos_ticlang\Debug
```

Clean command:

```powershell
& "D:\Ti\ccs2100\ccs\utils\bin\gmake" -k -j 32 clean -r -O
```

Build command:

```powershell
& "D:\Ti\ccs2100\ccs\utils\bin\gmake" -k -j 32 all -r -O
```

Build procedure:

Always run the clean command successfully before running the build command.
Do not run `all` alone, because stale linker or generated files may otherwise
be reused.

If the clean command is run from the repository root, `gmake` reports
`No rule to make target 'clean'` because there is no root-level makefile.

## Git

Always write commit messages in English.
