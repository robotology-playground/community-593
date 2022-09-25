// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#ifndef HELPERS_H
#define HELPERS_H

#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

class ObjectRetriever
{
    yarp::os::RpcClient portLocation;
    yarp::os::RpcClient portCalibration;
    bool calibrate(yarp::sig::Vector &location);

public:
    ObjectRetriever();
    bool getLocation(yarp::sig::Vector &location);
    virtual ~ObjectRetriever();
};

#endif
