#!/usr/bin/env bash
# Syntax-check Arcade-AtariG1.sv, which is NOT part of the normal Verilator
# lint (it instantiates MiSTer framework modules we don't have sources for).
# Catches the Quartus-only parse failures: missing commas, unbalanced blocks.
# Does NOT catch chained part-selects like v[15:8][i] - Verilator allows those
# and Quartus does not (error 10768), so grep for them separately:
#     grep -n '\[[0-9]*:[0-9]*\]\[' Arcade-AtariG1.sv
cd "$(dirname "$0")"
mkdir -p .lintstub
[ -f .lintstub/build_id.v ] || printf '`define BUILD_DATE "000000"\n' > .lintstub/build_id.v
verilator --lint-only -sv -Wno-fatal -I.lintstub -Isys -Irtl -Irtl/video -Irtl/jsa2 \
    Arcade-AtariG1.sv 2>&1 | grep -E '^%Error' | grep -v -E -i 'cannot find|not found|exiting due to'
echo "wrapper parse: $( [ -z "$(verilator --lint-only -sv -Wno-fatal -I.lintstub -Isys -Irtl -Irtl/video -Irtl/jsa2 Arcade-AtariG1.sv 2>&1 | grep -E '^%Error' | grep -v -E -i 'cannot find|not found|exiting due to')" ] && echo CLEAN || echo ERRORS )"
