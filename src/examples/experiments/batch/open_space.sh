#!/bin/bash

### How it works for me ###
# in ARGoS folder run the following:
# ./src/examples/experiments/batch/open_space.sh /src/examples/experiments/batch closed_space.argos

if [ "$#" -ne 2 ]; then
    echo "Usage: open_space.sh (from src folder) <base_config_dir> <base_config_file_name>"
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

res_dir=$wdir/"results/open_space"
if [[ ! -e $res_dir ]]; then
    mkdir $res_dir
    echo "mkdir: directory '$res_dir' "
else
    echo "Error: directory '$res_dir' already exists" 
    exit 1
fi

base_dir=`dirname $base_config`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1


#################################
# experiment_length is in seconds
#################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
experiment_type="open_space"
target_distance="0.5"
kilobot_distance="0.25"

# TEST
# experiment_length="180"
# RUNS=1
# numrobots="10"
# levy="1.2 1.4"
# crw="0.0 0.3"

# FINAL
RUNS=100
numrobots="10 20 50 100"
levy="1.2 1.4 1.6 1.8 2.0"
crw="0.0 0.3 0.6 0.9"

arenaSize="40, 40, 4"

for nrob in $numrobots; do
    for par1 in $levy; do
        for par2 in $crw; do
        param_dir=$res_dir/$date_time"_robots#"$nrob"_alpha#"$par1"_rho#"$par2"_"$experiment_length
        if [[ ! -e $param_dir ]]; then
            mkdir $param_dir
        fi

            for it in $(seq 1 $RUNS); do

                config=`printf 'config_nrob%d_levy%02d_crw%03d_seed%03d.argos' $nrob $par1 $par2 $it`
                echo config $config
                cp $base_config $config
                sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
                sed -i "s|__SEED__|$it|g" $config
                sed -i "s|__CRW__|$par2|g" $config
                sed -i "s|__LEVY__|$par1|g" $config
                sed -i "s|__EXPERIMENT__|$experiment_type|g" $config
                sed -i "s|__TARGETDISTANCE__|$target_distance|g" $config
                sed -i "s|__KILODISTANCE__|$kilobot_distance|g" $config
                sed -i "s|__ARENASIZE__|$arenaSize|g" $config
                sed -i "s|__NUMROBOTS__|$nrob|g" $config
                
                timeStats_file="seed#${it}_timeStats.txt"
                sed -i "s|__TIMESTATS__|$timeStats_file|g" $config

                kilo_file="seed#${it}_kiloLOG.txt"
                sed -i "s|__KILOLOG__|$kilo_file|g" $config

                
                echo "Running next configuration Robots $nrob LEVY $par1 CRW $par2"
                echo "argos3 -c $1$config"
                argos3 -c './'$config
            mv $timeStats_file $param_dir && mv $kilo_file $param_dir
            done
        done
    done
done

rm *.argos


