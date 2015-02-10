/home/jbot/milight_sources/milight 1 b 1
/home/jbot/milight_sources/milight 1 b 1
/home/jbot/milight_sources/milight 1 c 180
/home/jbot/milight_sources/milight 1 c 180
sleep 60

/home/jbot/milight_sources/milight 3 W
/home/jbot/milight_sources/milight 3 b 10

for i in `seq 2 9`; do
val=$((180-2*$i))
/home/jbot/milight_sources/milight 1 c $val
/home/jbot/milight_sources/milight 1 c $val
/home/jbot/milight_sources/milight 1 b $i
/home/jbot/milight_sources/milight 1 b $i
sleep 30
done

for i in `seq 10 16`; do
/home/jbot/milight_sources/milight 1 b $i
/home/jbot/milight_sources/milight 1 b $i
sleep 40
done

/home/jbot/milight_sources/milight 1 OFF
