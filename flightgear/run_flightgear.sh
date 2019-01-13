#!/bin/sh

FLIGHTGEAR_PATH="/Applications/FlightGear.app/Contents/MacOS/"

# --native=socket,direction,hz,machine,port,type

nice ${FLIGHTGEAR_PATH}/fgfs \
     --native-fdm=socket,in,20,,5432,udp \
     --disable-real-weather-fetch \
     --timeofday=noon \
     --disable-save-on-exit \
     --disable-terrasync \
     --browser-app=open \
     --aircraft=c172p \
     --fdm=external \
#    --fg-aircraft="$AUTOTESTDIR/aircraft" \
#    --units-meters \
#    --geometry=650x550 \
     --shading-flat \
     --bpp=32 \
     --prop:/sim/rendering/multi-sample-buffers=true\
     --prop:/sim/rendering/multi-samples=4\
     --prop:/environment/params/jsbsim-turbulence-model=ttCulp\
#    --timeofday=noon \
#    --disable-anti-alias-hud \
#    --disable-hud-3d \
     --disable-sound \
#    --disable-fullscreen \
#    --disable-random-objects \
#    --disable-anti-alias-hud \
#    --wind=0.0@0 \
     $*
