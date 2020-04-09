set mem inaccessible-by-default off
set architecture arm
target extended-remote /dev/ttyS9
monitor swdp_scan
b rust_begin_unwind
attach 1
load
compare-sections
