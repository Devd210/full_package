#!/usr/bin/env bash

ws_name=mowito_ws
recipe=""

while [[ $# -gt 0 ]]
do
	key="$1"

	case $key in
	    -w|--workspace)
	    ws_name="$2"
	    shift # past argument
	    shift # past value
	    ;;
	   	-l|--list)
		ls ~/$ws_name/src/mw_tools/rosinstalls/
		exit 0
	    shift # past argument
	    shift # past value
	    ;;
	    -h|--help)
		echo "usage: rosrun mw_tools update.sh recipe -w workspace_dir_name"
	    echo "-w --workspace default: mowito_ws | to give the name of the workspace"
	    echo "-l --list : to list all the available recipe/rosinstall files. Do use the extension 'rosinstall'"
	    exit
	    shift # past argument
	    shift # past value
	    ;;
	    *)    # unknown option
	    recipe="$1" # save it in an array for later
	    shift # past argument
	    ;;
	esac
done

if dpkg -s python-wstool > /dev/null
then
    echo " wstool installed"
else
    sudo apt install python-wstool
fi

cd ~/$ws_name

# updating the mw_tools
git -C ~/$ws_name/src/mw_tools pull

FILE=.rosinstall
if test -f "$FILE"; then
    echo " found the rosinstall "
else 
	wstool init
fi

echo "============ the recipe $recipe ===================="
if [ -z $recipe ]
then
	echo "no recipe or workspace provided, updating current recipe for $ws_name"	
	wstool update
else

	echo "updating to recipe $recipe in workspace of $ws_name"
	cp src/mw_tools/rosinstalls/$recipe.rosinstall .rosinstall
	wstool update
fi

