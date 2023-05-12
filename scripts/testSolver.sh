#!/bin/bash
SOLVER="CBSB"
SUBOPTIMALITY=1.2
PROBLEM_TYPE="FLOWTIME"
BYPASS=true
MAX_SOLVE_TIME=30.0
FILE_NAME=$PROBLEM_TYPE"_"$SOLVER"_w_"$SUBOPTIMALITY"_bp_"$BYPASS"_k"

echo "Solver="$SOLVER
echo "Suboptimality="$SUBOPTIMALITY
echo "Problem type="$PROBLEM_TYPE
echo "Bypass="$BYPASS
echo "Max solve time="$MAX_SOLVE_TIME
echo "Filename="$FILE_NAME

mkdir -p ../output/test
for i in {1..10..2}
do
	echo "num_agents: $i"
	for j in {1..1..1}
	do

	# Random Map
	../build/cbsb -m ../input/random/random-32-32-20.map -a ../input/random/scen-random/random-32-32-20-random-$j.scen -o ../output/test/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS

	# # Dragon Den
	# ../build/cbsb -m ../input/den/den520d.map -a ../input/den/scen-random/den520d-random-$j.scen -o  ../output/den/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS

	# # Paris
	# ../build/cbsb -m ../input/paris/Paris_1_256.map -a ../input/paris/scen-random/Paris_1_256-random-$j.scen -o ../output/paris/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS

	# # Warehouse
	# ../build/cbsb -m ../input/warehouse/warehouse-10-20-10-2-1.map -a ../input/warehouse/scen-random/warehouse-10-20-10-2-1-random-$j.scen -o ../output/warehouse/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS
	
    # # Empty
	# ../build/cbsb -m ../input/empty/empty-32-32.map -a ../input/empty/scen-random/empty-32-32-random-$j.scen -o ../output/test/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS

    # # Boston
	# ../build/cbsb -m ../input/boston/Boston_0_256.map -a ../input/boston/scen-random/Boston_0_256-random-$j.scen -o ../output/boston/$FILE_NAME$i.csv -k $i -t $MAX_SOLVE_TIME --solver=$SOLVER --suboptimality=$SUBOPTIMALITY --problemType=$PROBLEM_TYPE --bypass=$BYPASS
	done

done
