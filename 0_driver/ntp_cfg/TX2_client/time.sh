echo "1" | sudo -S service ntp stop & sleep 1;
echo "1" | sudo -S ntpdate -u i7 & sleep 8;
date & sleep 1;
echo "1" | sudo -S service ntp restart;
