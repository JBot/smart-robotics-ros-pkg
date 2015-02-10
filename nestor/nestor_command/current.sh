#!/bin/sh
#AccuWeather (r) RSS weather tool for conky
#
#USAGE: weather.sh <locationcode>
#
#(c) Michael Seiler 2007

METRIC=1 #Should be 0 or 1; 0 for F, 1 for C

if [ -z $1 ]; then
    echo
    echo "USAGE: weather.sh <locationcode>"
    echo
    exit 0;
fi

#curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/Currently/) {chomp;/\<title\>Currently: (.*)?\<\/title\>/; print "$1"; }'
#curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/Currently/) {chomp;/\<description\>Currently in Paris, FR:(.*)?/; print "$1 \n"; }'
#curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/description/) {chomp;/\<description\>High:(.*)?/; print "$1 \n"; }'
#curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/Forecast/) {chomp;/\<title\>(.*)?\<\/title\>/; print "$1"; }'
#curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/Currently/) {chomp;/\<description\> (.*)?\n/; print "$1"; }'

out=`curl -s http://rss.accuweather.com/rss/liveweather_rss.asp\?metric\=${METRIC}\&locCode\=$1 | perl -ne 'if (/Currently/) {chomp;/\<title\>Currently: (.*)?\<\/title\>/; print "$1"; }'`

#echo $out

cond=`echo $out | cut -d':' -f 1`
temp=`echo $out | cut -d':' -f 2`

case "$cond" in

'Fog')
    echo "Brumeux\c"
;;
'Snow')
    echo "Neigeux\c"
;;
'Cloudy')
    echo "Nuageux\c"
;;
'Sunny')
    echo "Ensoleillé\c"
;;
'T-Storms')
    echo "Orageux\c"
;;
'Clear')
    echo "Clair\c"
;;
'Mostly Cloudy')
    echo "Relativement nuageux\c"
;;
*)
    echo $cond'\c'
esac

echo : $temp
