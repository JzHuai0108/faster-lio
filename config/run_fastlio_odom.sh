#!/bin/bash

ws_dir="/home/pi/Documents/lidar/fasterlio_ws"
result_dir="/media/pi/BackupPlus/jhuai/results/fastlio"

lidarslam() {
cd $ws_dir
source devel/setup.bash
bagnames=("$@")
count=0
for bag in "${bagnames[@]}"; do
  echo "Processing bag: "$bag"_aligned.bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=$datadir/"$bag"_aligned.bag
  save_dir=$result_dir/$bag/
  echo "bagfile: $bagfile save_dir: $save_dir"
  mkdir -p $save_dir

  $ws_dir/devel/lib/faster_lio/run_mapping_offline --bag_file $bagfile --config_file \
    $ws_dir/src/faster-lio/config/hesai32_mti3dk_handheld.yaml --traj_log_file \
    $save_dir/scan_states.txt --time_log_file $save_dir/timing.log 2>&1 | tee $save_dir/fasterlio.log

  # roslaunch faster_lio mapping_hesai.launch bag_file:=$bagfile \
  #   config_yaml:=$ws_dir/src/faster-lio/config/hesai32_mti3dk_handheld.yaml \
  #   traj_log_file:=$save_dir/scan_states.txt time_log_file:=$save_dir/timing.log 2>&1 | tee $save_dir/fasterlio.log

  count=$((count+1))
#   if [ $count -eq 1 ]; then
#     break
#   fi
done
}

datadir="/media/pi/My_Book/jhuai/data/zongmu"
bags202401=(
  20240113/data1
  20240113/data2
  20240113/data3
  20240113/data4
  20240113/data5
  20240115/data1
  20240115/data2
  20240115/data3
  20240115/data4
  20240116/data2
  20240116/data3
  20240116/data4
  20240116/data5
  20240116_eve/data1
  20240116_eve/data2
  20240116_eve/data3
  20240116_eve/data4
  20240116_eve/data5)

bags202401=(
  20240116_eve/data5)
lidarslam "${bags202401[@]}"

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/zongmu"
bags202312=(
      20231201/data2
      20231201/data3
      # 20231208/data1
      20231208/data2
      20231208/data3
      20231208/data4
      20231208/data5
      20231213/data1
      20231213/data2
      20231213/data3
      20231213/data4
      20231213/data5)

# lidarslam "${bags202312[@]}"
