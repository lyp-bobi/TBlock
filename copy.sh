ip1=$1":~/tblock/"

echo $ip1
rsync -rv ./* ${ip1}
rsync -rv ./cmake ${ip1}/

ssh $1 "mkdir ~/tblock;mkdir ~/run;mkdir ~/build/tblock"
ssh $1 "rm ~/build/tblock/CMakeCache.txt"
ssh $1 "cd ~/build/tblock;cmake -DCMAKE_BUILD_TYPE=Debug ../../tblock/;make -j4"
ssh $1 -t 'cd ~/run;bash --login'
