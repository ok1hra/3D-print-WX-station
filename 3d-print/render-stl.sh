#!/bin/bash
for i in {1..17}	# part number
do
	D=$(date +%H:%M)
	echo "Render $i.stl    of 17 | $D"
#	/usr/bin/openscad -o stl-output/$i.stl wx-station.scad -D Part="$i" -D CUT="0"
	/home/dan/inst/OpenSCAD-2021.01-x86_64.AppImage -o stl-output/$i.stl wx-station.scad -D Part="$i" -D CUT="0"
done

exit 0
