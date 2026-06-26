#!/bin/bash
# Convert ROS 2 epoch log timestamps (seconds.nanoseconds) on stdin into a
# short, readable local clock time (HH:MM:SS).  Used by launcher.sh.
#   e.g.  [1782434179.406602296]  ->  [17:36:19]
Z=$(date +%z); SIGN=${Z:0:1}; H=${Z:1:2}; M=${Z:3:2}
OFF=$((10#$H*3600 + 10#$M*60)); [ "$SIGN" = "-" ] && OFF=$((-OFF))
exec stdbuf -oL awk -v OFF="$OFF" '
{
  line=$0; out=""
  while (match(line, /[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]\.[0-9]+/)) {
    tok=substr(line,RSTART,RLENGTH); dot=index(tok,".")
    epoch=substr(tok,1,dot-1)+0
    t=(epoch+OFF)%86400; if(t<0)t+=86400
    out=out substr(line,1,RSTART-1) sprintf("%02d:%02d:%02d",int(t/3600),int((t%3600)/60),t%60)
    line=substr(line,RSTART+RLENGTH)
  }
  print out line; fflush()
}'
