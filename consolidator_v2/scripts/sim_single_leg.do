# One-leg integration runner: one ASIC + one Tail + V2 + FT600Q.
# All commands traverse the USB TLM; absent Tail MISO pins are passive pulls.

set USE_PLL_STUB 1
set SIM_DEFINES "+define+RUN_SINGLE_LEG+USE_PLL_STUB"
if {!$USE_PLL_STUB} { set SIM_DEFINES "+define+RUN_SINGLE_LEG" }
set SIM_TB_FILE "tb_single_leg.sv"
set SIM_TOP "tb_single_leg"
set SIM_LOG "sim_single_leg.log"

# sim_aced.do owns the canonical RTL/file-list/model compile order.
do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim_aced.do"
