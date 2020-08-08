Sample command-line for running UP-DOWN code base:
-----------
./build/Garnet_standalone/gem5.opt configs/example/garnet_synth_traffic.py --topology=irregularMesh_XY --conf-file=64_nodes-connectivity_matrix_0-links_removed_12.txt --num-cpus=64 --num-dirs=64 --mesh-rows=8 --network=garnet2.0 --up-dn=1 --escape-vc=1 --sim-cycles=100000 --sim-type=2 --inj-vnet=0 --vcs-per-vnet=4 --injectionrate=0.10 --synthetic=uniform_random --routing-algorithm=0
-----------
Note:
parameter to vary:
--conf-file <= used to read the topology from the configuration file that gem5 will read and simulate
----
keep both
'--up-dn' and '--escape-vc' to 1.