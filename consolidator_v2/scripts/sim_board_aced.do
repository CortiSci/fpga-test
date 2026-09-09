# Extracted-board four-ASIC/four-Tail ACED integration runner.
set USE_PLL_STUB 1
set SIM_DEFINES "+define+RUN_ACED+USE_PLL_STUB"
if {!$USE_PLL_STUB} { set SIM_DEFINES "+define+RUN_ACED" }
set SIM_TB_FILE "tb_board.sv"
set SIM_TOP "tb_board"
set SIM_LOG "sim_board_aced.log"
do "C:/cortisci/IONM-A/IONM-A-FPGA/consolidator_v2/scripts/sim_aced.do"
