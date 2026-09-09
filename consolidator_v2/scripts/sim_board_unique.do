# Extracted-board four-ASIC/four-Tail unique identity integration runner.
set USE_PLL_STUB 1
set SIM_DEFINES "+define+RUN_UNIQUE+USE_PLL_STUB"
if {!$USE_PLL_STUB} { set SIM_DEFINES "+define+RUN_UNIQUE" }
set SIM_TB_FILE "tb_board.sv"
set SIM_TOP "tb_board"
set SIM_LOG "sim_board_unique.log"
do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim_aced.do"
