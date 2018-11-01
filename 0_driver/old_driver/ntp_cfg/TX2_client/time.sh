echo "dji" | sudo -S service ntp stop & sleep 1;
echo "dji" | sudo -S ntpdate -u 10.0.0.2 & sleep 8;
date & sleep 1;
echo "dji" | sudo -S service ntp restart;
