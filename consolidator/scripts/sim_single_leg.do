# Current-source Questa GUI entry point: consolidator_single_leg_stream.
# Load with do <this-file>; then run -all. See tools/ci/README.md.
set registry_repo [file normalize [file join [file dirname [status file]] .. .. ..]]
source [file join $registry_repo tools ci questa_launch.do]
launch_registry_target $registry_repo consolidator_single_leg_stream
