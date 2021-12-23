#!/bin/bash
set -e

ARGC=$#
if [ $ARGC -lt 1 ]
then
	echo "Usage: $0 -i=<Eingabedatei> [-o=<Ausgabedatei-Präfix>]"
	echo "[-rect=<x,y,width,height>] (Benutzt nur einen Bildausschnitt)"
	echo "[-t] (Erstellt Dreicke anstatt Vierecke)"
	echo "[-f] (Invertiert die Richtung der Flächen)"
	echo "[-c] (Center)"
	echo "[-b] (Binary PLY)"
	echo "[-tx] (Write texture)"
	echo "[-m<x,y,z>] (Spiegelt die entsprechende Achse)"
	echo "[-r] (Erzwingt die neuberechnung der XYZ Daten)"
	echo "[-xyz] (Speichert die XYZ Daten)"
	exit 1
fi


# Defaults
F_TRIANGLES=""
F_INVERT_FACES=""
F_MIRROR_X=""
F_MIRROR_Y=""
F_MIRROR_Z=""
F_CENTER=""
F_BINARY=""
F_RECALCULATE="false"
F_KEEP_XYZ="false"
F_WRITE_TEXTURE="false"

# Args parsen
for ARG in "$@"
do

 case "$ARG" in
  -i=*)
    IPF="${ARG#*=}"
    ;;
  -o=*)
    OPF="${ARG#*=}"
    ;;
  -rect=*)
    RECT="${ARG#*=}"
    ;;
  -t)
    F_TRIANGLES="-t"
    ;;
  -f)
    F_INVERT_FACES="-f"
    ;;
  -mx)
    F_MIRROR_X="-mx"
    ;;
  -my)
    F_MIRROR_Y="-my"
    ;;
  -mz)
    F_MIRROR_Z="-mz"
    ;;
  -c)
    F_CENTER="-c"
    ;;
  -b)
    F_BINARY="-b"
    ;;
  -r)
    F_RECALCULATE="true"
    ;;
  -xyz)
    F_KEEP_XYZ="true"
    ;;
  -tx)
    F_WRITE_TEXTURE="true"
    ;;
 esac

done

# Prüfe auf gültige Eingabedatei
if [ -z "$IPF"  ] || [ ! -f "$IPF" ]
then
  echo "Keine gültige Eingabedatei: '$IPF'"
  exit 1
fi

# Berechnete Variablen
OPF_PRE="${OPF:-$IPF}";
OPF_XYZ="$OPF_PRE.xyz"
OPF_XYZ_RECT="$OPF_PRE.rect.xyz"
OPF_PLY="$OPF_PRE.ply"
OPF_TEX=""
PIXEL_WIDTH=$( { identify -format '%w' "$IPF"; } 2>/dev/null )
PIXEL_WIDTH_RECT="$PIXEL_WIDTH"

echo "Convert $IPF -> $OPF_XYZ -> $OPF_PLY"
echo ""

# Erstelle XYZ Datei
if [ ! -f "$OPF_XYZ" ] || [ "$F_RECALCULATE" = "true" ]
then
  echo "gdal_translate -of XYZ \"$IPF\" \"$OPF_XYZ\""
  gdal_translate -of XYZ "$IPF" "$OPF_XYZ"
  echo ""
fi

if [ "$F_WRITE_TEXTURE" = "true" ]
then
  OPF_TEX="$OPF_PRE.jpg"

  if [ -z "$RECT" ]
  then
    convert "$IPF" "$OPF_TEX" 2>/dev/null
  else
    RECT_ARR=(${RECT//,/ })
    convert "$IPF" -crop "${RECT_ARR[2]}x${RECT_ARR[3]}+${RECT_ARR[0]}+${RECT_ARR[1]}" +repage "$OPF_TEX" 2>/dev/null
  fi

fi

if [ -z "$RECT" ]
then
  cp "$OPF_XYZ" "$OPF_XYZ_RECT"
else
  ./XYZ2RECT.groovy -i "$OPF_XYZ" -p "$PIXEL_WIDTH" -o "$OPF_XYZ_RECT" -r "$RECT"
  RECT_ARR=(${RECT//,/ })
  PIXEL_WIDTH_RECT="${RECT_ARR[2]}"
fi

# java <self> <*.xyz input file> <pixel width>
# [*.ply output file] [triangles - true|false] [invert faces - true|false] [mirrorXYZ - true|false]x3
if [ "$F_WRITE_TEXTURE" = "true" ] && [ -f "$OPF_TEX" ]
then
  ./XYZ2PLY.groovy -i "$OPF_XYZ_RECT" -p "$PIXEL_WIDTH_RECT" -o "$OPF_PLY" -tx "$OPF_TEX" "$F_CENTER" "$F_TRIANGLES" "$F_INVERT_FACES" "$F_MIRROR_X" "$F_MIRROR_Y" "$F_MIRROR_Z" "$F_BINARY"
else
  ./XYZ2PLY.groovy -i "$OPF_XYZ_RECT" -p "$PIXEL_WIDTH_RECT" -o "$OPF_PLY" "$F_CENTER" "$F_TRIANGLES" "$F_INVERT_FACES" "$F_MIRROR_X" "$F_MIRROR_Y" "$F_MIRROR_Z" "$F_BINARY"
fi


if [ "$F_KEEP_XYZ" = "false" ]
then
  rm -f "$OPF_XYZ"
fi
