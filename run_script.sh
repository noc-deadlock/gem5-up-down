#!/bin/bash


#### script for running escapeVC ####

benchmarks=( 'BC' 'BellmanFord' 'BFS' 'BFS-Bitvector' 'BFSCC' 'CF' 'Components' 'Components-Shortcut' 'KCore' 'MIS' 'PageRank' 'PageRankDelta' 'Radii' 'Triangle' )
inputs=( 'rMatGraph_J_5_100' 'rMatGraph_WJ_5_100' )

num_cores=64
file=$1
d="07-26-2019"
out_dir="/usr/scratch/mayank/ligra_benchmark_result/irregular_up_dn/$d/UP_DN"
up_dn_=1
esc_vc_=0
vc_=$2


# commandlines for benchmarks
for b in 0 2 3 4 6 7 8 9 10 11 12 13
do
	./build/RISCV_MESI_Two_Level/gem5.opt -d $out_dir/${benchmarks[$b]}/${file}/vc-${vc_} --remote-gdb-port=0 configs/example/se.py --cpu-type TimingSimpleCPU --num-cpus=64 --l1d_size=16kB --l1i_size=16kB --l1d_assoc=4 --num-l2caches=64 --l2_size=128kB --l2_assoc=8 --num-dirs=4 --ruby --mem-size=4096MB --network=garnet2.0 --topology=irregularMesh_XY --mesh-rows=8 --up-dn=1 --escape-vc=0 --vcs-per-vnet=${vc_} --conf-file=${file} -c my_benchmarks/ligra/bin/riscv/${benchmarks[$b]} -o '-n 64 my_benchmarks/ligra/input/rMatGraph_J_5_100' &
done

# 'BellmanFord' and 'CF' application uses a different input
for b in 1 5
do
	./build/RISCV_MESI_Two_Level/gem5.opt -d $out_dir/${benchmarks[$b]}/${file}/vc-${vc_} --remote-gdb-port=0 configs/example/se.py --cpu-type TimingSimpleCPU --num-cpus=64 --l1d_size=16kB --l1i_size=16kB --l1d_assoc=4 --num-l2caches=64 --l2_size=128kB --l2_assoc=8 --num-dirs=4 --ruby --mem-size=4096MB --network=garnet2.0 --topology=irregularMesh_XY --mesh-rows=8 --up-dn=1 --escape-vc=0 --vcs-per-vnet=${vc_} --conf-file=${file} -c my_benchmarks/ligra/bin/riscv/${benchmarks[$b]} -o '-n 64 my_benchmarks/ligra/input/rMatGraph_WJ_5_100' &
done
