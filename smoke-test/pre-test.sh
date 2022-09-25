#!/bin/bash

# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# Put here those instructions we need to execute before running the test

local_context_path=~/.local/share/yarp/contexts
context="simConfig"
file="iCub_parts_activation.ini"

abspath_to_file=${local_context_path}/${context}/${file}

if [ -f ${abspath_to_file} ]; then
    mv ${abspath_to_file} ${abspath_to_file}.bak
fi

# import default values
mkdir -p ${local_context_path}/${context}
cp ${ROBOTOLOGY_SUPERBUILD_INSTALL_PREFIX}/share/iCub/contexts/${context}/${file} ${abspath_to_file}

# enable objects
sed -i '/objects/c\objects on' ${abspath_to_file}

# enable skin
sed -i '/pressure/c\pressure on' ${abspath_to_file}
