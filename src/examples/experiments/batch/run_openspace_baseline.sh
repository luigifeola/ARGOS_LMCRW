#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: run_baseline.sh (from src folder) <base_config_dir> <base_config_file_name>"
    exit 11
fi

wdir=`pwd`
base_config=$1$2
if [ ! -e $base_config ]; then
    base_config=$wdir/$1/$2
    if [ ! -e $base_config ]; then
        echo "Error: missing configuration file '$base_config'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/baseline_openspace"
if [[ ! -e $res_dir ]]; then
    mkdir $res_dir
else
    echo "Error: directory '$res_dir' already exists" 
    exit 1
fi

base_dir=`dirname $base_config`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1

levy="1.2 1.6 2.0"
crw="0.0 0.3 0.6 0.9"
bias_prob="0.1"
numrobots="1"
numWalls="0"
arenaSize="40, 40, 4"
radius="0.0"
#################################
# experiment_length is in seconds
#################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
RUNS=500


for par1 in $levy; do
    for par2 in $crw; do
	param_dir=$res_dir/$date_time"_robots#"$numrobots"_alpha#"$par1"_rho#"$par2"_baseline_"$experiment_length
	if [[ ! -e $param_dir ]]; then
	    mkdir $param_dir
	fi

        for it in $(seq 1 $RUNS); do

            config=`printf 'config_levy%02d_crw%03d_seed%03d.argos' $par1 $par2 $it`
            echo config $config
            cp $base_config $config
            sed -i "s|__NUMROBOTS__|$numrobots|g" $config
            sed -i "s|__BIASPROB__|$bias_prob|g" $config
            sed -i "s|__RADIUS__|$radius|g" $config
            sed -i "s|__NUMWALLS__|$numWalls|g" $config
            sed -i "s|__ARENASIZE__|$arenaSize|g" $config
            sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
            sed -i "s|__SEED__|$it|g" $config
            sed -i "s|__CRW__|$par2|g" $config
            sed -i "s|__LEVY__|$par1|g" $config
            output_file="seed#${it}_time_results.tsv"
            sed -i "s|__OUTPUT__|$output_file|g" $config

            positions_file="seed#${it}_position.tsv"
            sed -i "s|__POSOUTPUT__|$positions_file|g" $config

            
            echo "Running next configuration LEVY $par1 CRW $par2"
            echo "argos3 -c $1$config"
            argos3 -c './'$config
	    mv $output_file $param_dir && mv $positions_file $param_dir
        done
    done
done

rm *.argos


