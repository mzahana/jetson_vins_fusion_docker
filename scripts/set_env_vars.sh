#!/bin/bash
# Author Mohamed Abdelkader, mohamedashraf123@gmail.com

FILE_NAME="vins_env_vars.txt"

if [ -z "$1" ]
  then
    echo "Environment variables txt file name is not passed. Using vins_env_vars.txt"
else
	FILE_NAME="$1"
fi
echo "Environment varilable file name = $FILE_NAME"

if [ -d "$HOME/jetson_vins_fusion_docker" ]; then
    file="$HOME/jetson_vins_fusion_docker/scripts/$FILE_NAME"
elif [ -d "$HOME/src/jetson_vins_fusion_docker" ]; then
    file="$HOME/src/jetson_vins_fusion_docker/scripts/$FILE_NAME"
else
    echo "ERROR Could not find jetson-containers package. Exiting" && echo
    exit 1
fi
# TODO if the env_vars.txt is empty, through an error and exit

# Read the env_vars.txt file and copy the environment variables to $HOME/.bashrc
export cmd_str=''
while read -r line
do
	# skip empty lines
	if [ ! -z "$line" ]; then
		[[ "$line" =~ ^#.*$ ]] && continue
		# Extract variable name
		var_name=$(echo $line| cut -d'=' -f 1)
		# Extract variable value
		var_value=$(echo $line| cut -d'=' -f 2)

		#echo "$var_name=$var_value"

	      export $var_name=$(eval echo $var_value)
	      export cmd_str="$cmd_str; export $var_name=$(eval echo $var_value)"
	fi
#    sed -i "/$var_name/d" $HOME/.bashrc
#    echo "export $var_name=${var_value}" >> $HOME/.bashrc
done <"$file"
#echo $cmd_str
