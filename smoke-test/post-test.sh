#!/bin/bash

# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# Put here those instructions we need to execute after having run the test

local_context_path=~/.local/share/yarp/contexts
context="simConfig"
file="iCub_parts_activation.ini"

abspath_to_file=${local_context_path}/${context}/${file}

if [ -f ${abspath_to_file} ]; then
    rm ${abspath_to_file}
    
    if [ -f ${abspath_to_file}.bak ]; then
        mv ${abspath_to_file}.bak ${abspath_to_file}
    fi    
fi
