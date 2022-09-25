/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#include <string>

#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>

#include <yarp/robottestingframework/TestCase.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

/**********************************************************************/
class TestAssignmentMakeItRoll : public yarp::robottestingframework::TestCase,
                                 public PortReader
{
    RpcClient portBall;
    RpcClient portMIR;
    Port      portHand;

    Vector ballPosRobFrame;
    bool hit;

    /******************************************************************/
    Vector getBallPosition()
    {
        Bottle cmd,reply;
        cmd.addString("world");
        cmd.addString("get");
        cmd.addString("ball");
        if (!portBall.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to world");
        if (reply.size()<3)
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Invalid reply from world");

        Vector pos(3);
        pos[0]=reply.get(0).asFloat64();
        pos[1]=reply.get(1).asFloat64();
        pos[2]=reply.get(2).asFloat64();

        return pos;
    }

    /******************************************************************/
    bool setBallPosition(const Vector& pos)
    {
        if (pos.length()>=3)
        {
            Bottle cmd,reply;
            cmd.addString("world");
            cmd.addString("set");
            cmd.addString("ball");
            cmd.addFloat64(pos[0]);
            cmd.addFloat64(pos[1]);
            cmd.addFloat64(pos[2]);
            if (!portBall.write(cmd,reply))
                ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to world");
            return true;
        }
        else
            return false;
    }

public:
    /******************************************************************/
    TestAssignmentMakeItRoll() :
        yarp::robottestingframework::TestCase("TestAssignmentMakeItRoll"),
        hit(false)
    {
    }

    /******************************************************************/
    virtual ~TestAssignmentMakeItRoll()
    {
    }

    /******************************************************************/
    virtual bool setup(yarp::os::Property& property)
    {
        string robot=property.check("robot",Value("icubSim")).asString();
        string hand=property.check("hand",Value("right")).asString();
        float rpcTmo=(float)property.check("rpc-timeout",Value(120.0)).asFloat64();

        string robotPortName("/"+robot+"/cartesianController/"+hand+"_arm/state:o");

        string portBallName("/"+getName()+"/ball:rpc");
        string portMIRName("/"+getName()+"/mir:rpc");
        string portHandName("/"+getName()+"/hand:i");

        portBall.open(portBallName);
        portMIR.open(portMIRName);
        portHand.open(portHandName);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Set rpc timeout = %g [s]",rpcTmo));
        portBall.asPort().setTimeout(rpcTmo);
        portMIR.asPort().setTimeout(rpcTmo);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Connecting Ports");

        if (!Network::connect(portBallName,"/icubSim/world"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /icubSim/world");

        if (!Network::connect(portMIRName,"/service"))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to connect to /service");

        if (!Network::connect(robotPortName,portHandName))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL(Asserter::format("Unable to connect to %s",robotPortName.c_str()));

        Rand::init();

        return true;
    }

    /******************************************************************/
    virtual void tearDown()
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Closing Ports");
        portBall.close();
        portMIR.close();
        portHand.close();
    }

    /******************************************************************/
    virtual bool read(ConnectionReader& reader)
    {
        if (!hit)
        {
            Bottle data;
            data.read(reader);

            Vector x(3);
            x[0]=data.get(0).asFloat64();
            x[1]=data.get(1).asFloat64();
            x[2]=data.get(2).asFloat64();

            double d=norm(ballPosRobFrame-x);
            if (d<0.05)
            {
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Great! We're at %g [m] from the ball",d));
                hit=true;
            }
        }

        return true;
    }

    /******************************************************************/
    void finishup(const unsigned int score, const string &msg)
    {
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(msg);
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(false,Asserter::format("Total score = %d",score));
    }

    /******************************************************************/
    virtual void run()
    {
        Time::delay(5.0);

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Retrieving initial ball position");
        Vector initialBallPos=getBallPosition();
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("initial ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        Vector min(3,0.0),max(3,0.0);
        min[0]=-0.02; max[0]=0.04;  // x-axis
        min[1]= 0.0;  max[1]=0.0;   // y-axis
        min[2]=-0.03; max[2]=0.03;  // z-axis

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Setting new initial ball position");
        initialBallPos+=Rand::vector(min,max);
        setBallPosition(initialBallPos);
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("new ball position = (%s) [m]",
                                         initialBallPos.toString(3,3).c_str()));

        // compute ball position in robot's root frame
        Matrix T=zeros(4,4);
        T(0,1)=-1.0;
        T(1,2)=1.0;  T(1,3)=0.5976;
        T(2,0)=-1.0; T(2,3)=-0.026;
        T(3,3)=1.0;
        Vector initBallPosHomog=initialBallPos;
        initBallPosHomog.push_back(1.0);
        ballPosRobFrame=SE3inv(T)*initBallPosHomog;
        ballPosRobFrame.pop_back();

        unsigned int score=0;

        Bottle cmd,reply;
        cmd.addString("look_down");
        if (!portMIR.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to MIR");
        if (reply.get(0).asString()!="ack")
        {
            finishup(0,"Unable to look_down");
            return;
        }
        cmd.clear(); reply.clear();

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("We looked down! Gained 1 point");
        score++;

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Proximity check is now active");
        portHand.setReader(*this);

        cmd.addString("make_it_roll");
        if (!portMIR.write(cmd,reply))
            ROBOTTESTINGFRAMEWORK_ASSERT_FAIL("Unable to talk to MIR");
        if (reply.get(0).asString()!="ack")
        {
            finishup(score,"Unable to make_it_roll");
            return;
        }
        cmd.clear(); reply.clear();

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("We tried to roll! Gained 1 point");
        score++;

        ROBOTTESTINGFRAMEWORK_TEST_REPORT("Retrieving final ball position");
        Vector finalBallPos=getBallPosition();
        ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("final ball position = (%s) [m]",
                                         finalBallPos.toString(3,3).c_str()));

        if (hit)
        {
            ROBOTTESTINGFRAMEWORK_TEST_REPORT("We hit the ball! Gained 2 point");
            score+=2;
        }

        double d=norm(finalBallPos-initialBallPos);
        if (d>0.01)
        {
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("Ball has rolled for at least %g [m]! Gained 3 points",d));
            score+=3;
        }

        ROBOTTESTINGFRAMEWORK_TEST_CHECK(true,Asserter::format("Total score = %d",score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentMakeItRoll)
