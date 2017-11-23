/***************************************************************************

    file                 : test_bot.cpp
    created              : Do 2. Nov 08:49:38 CET 2017
    copyright            : (C) 2002 Jonas Natzer, Michael Heinrich

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <tmath/linalg_t.h>
#include <iomanip>

#include "optimal_line.h"

#define MAX_BREAK_G 1.60
#define MAX_LATERAL_G 1.60
#define SUCTION_G_PER_M_SS (0.5 / 1000.0)

#define COLLISION_WARNING_DIST 7
#define COLLISION_AVOID_GAIN 10
#define OBSTRUCT_OP_GAIN -10

#define UNSTUCKING_STEPS 90
#define MANEUVER_INNOVATION 0.1

static tTrack *curTrack;

static bool isObstructing = false;
static float lastManeuver = 0;

static TrackModel trackModel;
static std::time_t lastDebugOutTime = std::time(NULL);
static int lastMoveStep = 0;
static int currentStep = 0;
static int remainingBackwardSteps = 0;

static bool hasLaunched = false;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);

/* 
 * Module entry point  
 */
extern "C" int
test_bot(tModInfo *modInfo)
{
    memset(modInfo, 0, 10 * sizeof (tModInfo));

    modInfo->name = strdup("Hemic"); /* name of the module (short) */
    modInfo->desc = strdup("2017 TU München research project by "
            "Michael Heinrich and Jonas Natzer"); /* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt; /* init function */
    modInfo->gfId = ROB_IDENT; /* supported framework version */
    modInfo->index = 1;

    return 0;
}

/* Module interface initialization. */
static int
InitFuncPt(int index, void *pt)
{
    tRobotItf *itf = (tRobotItf *) pt;

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    /* for every track change or new race */
    itf->rbNewRace = newrace; /* Start a new race */
    itf->rbDrive = drive; /* Drive during race */
    itf->rbPitCmd = NULL;
    itf->rbEndRace = endrace; /* End of the current race */
    itf->rbShutdown = shutdown; /* Called before the module is unloaded */
    itf->index = index; /* Index used if multiple interfaces */
    return 0;
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    curTrack = track;

    // track, base, blab, suctionParam
    trackModel.initialize(
                          track,
                          MAX_BREAK_G, // max Break G
                          MAX_LATERAL_G, // max lateral G
                          SUCTION_G_PER_M_SS // additional G per m/s (through suction)
                          );
    *carParmHandle = NULL;
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{
    std::cout << "test_bot.cpp: Version 0.07\n";
    
    hasLaunched = false;
}

/* Drive during race. */

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{
    memset(&car->ctrl, 0, sizeof (tCarCtrl));

    float angle;
    const float SC = 1.0;

    tCarElt *closestOponent = 0x00;
    float closestOpDist = 100;
    int ncars = s->raceInfo.ncars;

    tdble cX, cY, oX, oY;
    v2t<float> opponentDir;

    RtTrackLocal2Global(&(car->_trkPos), &cX, &cY, 0);

    for (int i = 0; i < ncars; i++)
    {
        tCarElt *op = s->cars[i];

        if (op == car)
        {
            continue;
        }

        if (op != 0x00)
        {
            RtTrackLocal2Global(&(op->_trkPos), &oX, &oY, 1);
            v2t<float> dif(oX - cX, oY - cY);
            float len = dif.len();



            if (len < closestOpDist)
            {
                closestOpDist = len;
                closestOponent = op;
                opponentDir = dif;
            }
        }
    }

    float maneuver = (1 - MANEUVER_INNOVATION) * lastManeuver;

    if (closestOponent != 0x00 && closestOpDist < COLLISION_WARNING_DIST)
    {
        float ownPos = car->_trkPos.toLeft;
        float otherPos = closestOponent->_trkPos.toLeft;

        float diff = ownPos - otherPos;
        float dir = diff;
        dir = dir >= 0 ? 1 : dir;
        dir = dir < 0 ? -1 : dir;

        bool isBehind = car->race.pos < closestOponent->race.pos;



        float intrusion = dir * (COLLISION_WARNING_DIST - closestOpDist) / COLLISION_WARNING_DIST;
        
        if (isBehind || isObstructing)
        {
            // obstruct car behind instead of avoidance;
            isObstructing = true;
            maneuver += -MANEUVER_INNOVATION * OBSTRUCT_OP_GAIN * intrusion;
        }
        else
        {
            // avoid car infront
            maneuver += MANEUVER_INNOVATION * COLLISION_AVOID_GAIN * intrusion;
        }

        /*
        std::cout << "COLLISION_WARNING: intrusion=" << intrusion <<
                ", maneuver=" << maneuver <<
                ", obstructManeuver=" << isBehind
                << "\n";
        */
    }
    else
    {
        // only stop obstruction mode when cars are out of range, so we wont
        // suddenly switch hard to avoidance if we are overtaken.
        isObstructing = false;
    }

    angle = trackModel.getTangentAngle(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

    float offset = trackModel.getOffsetFromCenter(&(car->_trkPos)) + maneuver;
    
    // half of width minus security margin
    float wh = 0.20 * (car->_trkPos.seg->startWidth + car->_trkPos.seg->endWidth);
    
    offset = offset > wh ? wh : offset;
    offset = offset < -wh ? -wh : offset;
    
    float correctiveAngle = -(SC * (car->_trkPos.toMiddle + offset)) / car->_trkPos.seg->width;

    if (correctiveAngle > .2)
    {
        correctiveAngle = .2;
    }

    if (correctiveAngle < -.2)
    {
        correctiveAngle = -.2;
    }

    angle += correctiveAngle;


    float speed = car->pub.speed;
    float speedLim = trackModel.getMaximumSpeed(&(car->_trkPos));

    // set up the values to return
    car->ctrl.steer = angle / car->_steerLock;

    float dv = speed - speedLim;

    if (dv < -2)
    {
        car->ctrl.accelCmd = 1.0; // 100% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if (dv < -1)
    {
        // dv between -2 and -1
        float cmd = -(dv + 1);
        
        car->ctrl.accelCmd = cmd; // linear accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if (dv < 0)
    {
        // dv between -1 and 0
        car->ctrl.accelCmd = 0.0; // no accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
    else if(dv < 1)
    {
        // dv between 0 and 1
        float cmd = dv;
        
        car->ctrl.accelCmd = 0.0;   // no accelerator pedal
        car->ctrl.brakeCmd = cmd;     // linear brakes
    }
    else
    {
        car->ctrl.accelCmd = 0.0; // no accelerator pedal
        car->ctrl.brakeCmd = 1; // 100% brakes
    }

    float maxRpm = car->_enginerpmRedLine * .9f;
    float minRpm = maxRpm * .6;

    int gear = car->_gear;

    if (car->_enginerpm > maxRpm)
    {
        gear++;
    }
    else if (car->_enginerpm < minRpm)
    {
        gear--;
    }

    if (gear > 6)
    {
        gear = 6;
    }

    if (gear < 1)
    {
        gear = 1;
    }

    float slip =
            car->priv.wheel[0].slipAccel +
            car->priv.wheel[1].slipAccel +
            car->priv.wheel[2].slipAccel +
            car->priv.wheel[3].slipAccel;
    
    slip *= 0.25;

    float skid = car->priv.skid[0] + car->priv.skid[1] +
            car->priv.skid[2] + car->priv.skid[3];

    skid *= 0.25;
    
    /*
    std::cout <<
            "skid=" << skid <<
            ", gear=" << gear <<
            ", slip=" << slip <<
            "\n";
    */

    if(slip < -10)
    {
        car->ctrl.accelCmd = 0;
    }
    else if(slip < -1)
    {
        float aSlip = -slip;
        float rel = (aSlip - 1.0) / (10.0 - 1.0);
        // smooth reduction of throttle on starting slip
        car->ctrl.accelCmd = rel * 0.0f + (1.0f - rel) * 1.0;
    }
    
    car->ctrl.gear = gear;
    
    
    bool moves = speed > 1;
    
    if(moves)
    {
        lastMoveStep = currentStep;
        hasLaunched = true;
    }
    else if(currentStep - lastMoveStep > 20 && hasLaunched)
    {
        remainingBackwardSteps = UNSTUCKING_STEPS;
    }
    
    if(remainingBackwardSteps > 0)
    {
        car->ctrl.gear = -1;
        car->ctrl.accelCmd = .3;
        car->ctrl.brakeCmd = 0;
        car->ctrl.steer = -angle / car->_steerLock;
        
        remainingBackwardSteps--;
    }
    
    if(!hasLaunched)
    {
        car->ctrl.gear = 1;
        car->ctrl.accelCmd = .6;
        car->ctrl.brakeCmd = 0;
    }

    std::time_t t = std::time(NULL);
    std::time_t dt = t - lastDebugOutTime;

    if (true || t >= 1)
    {
        lastDebugOutTime = t;

        //std::ofstream logfile;
        //logfile.open ("~/test_bot.log", std::ofstream::out | std::ofstream::app);


        std::cout << std::showpos << std::setprecision(3) << std::fixed
                << "off=" << std::setw(3) << offset
                << std::noshowpos
                << ", acc=" << std::setw(3) << car->ctrl.accelCmd
                << ", brk=" << std::setw(3) << car->ctrl.brakeCmd
                << ", spd=" << std::setw(3) << speed
                << ", spdLim=" << std::setw(3) << speedLim
                //<< ", friction=" << car->_trkPos.seg->surface->kFriction
                << "\n";

        
        lastManeuver = maneuver;
        //logfile.close();
    }

    
    currentStep++;
    /*
    car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
    car->ctrl.brakeCmd = 0.0; // no brakes
     */
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}


