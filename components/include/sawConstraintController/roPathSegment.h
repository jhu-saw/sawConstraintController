/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
$Id: $

Author(s):  Marcin Balicki
Created on: 2014

(C) Copyright 2006-2014 Johns Hopkins University (JHU), All Rights
Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

#ifndef _roPathSegment_h
#define _roPathSegment_h

#include <iostream>
#include <sstream>
#include <fstream>
#include <cisstVector/vctFixedSizeVectorTypes.h>

// This is PathSegment

class roPathSegment
{

public:
    roPathSegment(vct5 a, vct5 b, int index);
    roPathSegment(){};
    ~roPathSegment(){};
  
    vct5 start;
    vct5 end;
  
  
    /// return true if input is projected within the bounds of the "line"
    /// inPoseVec is the pose (aka point)
    /// transToRotFactor *r = t
    /// returns:
    ///   poseOnSeg (aka point) closest one to the given segment
    ///   distToClosestPoint  the weighted distance from input to closest point  (with factor)
    ///   distAlongVecToClosestPoint  = GetParametrization of the segment (how far along it are we). (with factor)
    // t is the var name used by RHT
  
    bool GetClosestPoint(const vct5 &inPoseVec,
                         const double &rotToTransFactor,
                         vct5 &poseOnSeg,
                         double & distToClosestPointWeighted,
                         double & normDistAlongVecToClosestPoint);
  
  
    int GetIndex(void){
      return Index;
    }
    
    int SetIndex(int index){
      Index = index;
      return Index;
    }
  

    static void Test();


private:
    int Index;
  
    //GetParametrization of the segment (how far along it are we).
    // t is the var name used by RHT
//    double GetNormalizedDistAlongVector(vct5 a);

  
//    double GetDistance(vct5 a);
    //depricated.
//    vct5 GetClosestPointOnLine(vct5 a);
//    vct5 GetClosestPointOnLine(vct5 a,  double transToRotFactor);

};

#endif
