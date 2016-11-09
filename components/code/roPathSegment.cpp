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

#include <sawConstraintController/roPathSegment.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstCommon/cmnConstants.h>

roPathSegment::roPathSegment(vct5 a, vct5 b, int index)
{
    start.Assign(a);
    end.Assign(b);
    Index = index;
}

///find the closest pose on a segment to the input pose using
///rotation/translation weighting
/// return true if the projection is within the segment limits
bool roPathSegment::GetClosestPoint(const vct5 &inPoseVec,
                                   const double &rotToTransFactor,
                                   vct5 &poseOnSeg,
                                   double & distToClosestPointWeighted,
                                   double & normDistAlongVecToClosestPoint)
{
  
  vct5 s = start;
  vct5 e = end;
  
  vct5 inWeighted = inPoseVec;
  
  s[3] *= rotToTransFactor;
  s[4] *= rotToTransFactor;

  e[3] *= rotToTransFactor;
  e[4] *= rotToTransFactor;

  inWeighted[3] *= rotToTransFactor;
  inWeighted[4] *= rotToTransFactor;
  
  vct5 esDiff = e - s;
  
  double numer = vctDotProduct(inWeighted - s, esDiff);
  double denom = esDiff.NormSquare();
  
  normDistAlongVecToClosestPoint = numer/denom;
  
  vct5 closestWeightedPose = s + esDiff * normDistAlongVecToClosestPoint;
  
  bool onEnd = false; //was out of range
  
  if(normDistAlongVecToClosestPoint < 0.0) {
    closestWeightedPose = s;
    onEnd = true;
  }
  else if(normDistAlongVecToClosestPoint > 1.0) {
    closestWeightedPose = e;
    onEnd = true;
  }

  //onEnd = true;
  
  /// compute the weighted distance from the input pose to the closest pose
  distToClosestPointWeighted = (closestWeightedPose - inWeighted).Norm();
  
  /// compute the unweighted closest pose
  poseOnSeg = closestWeightedPose;
  poseOnSeg[3] /= rotToTransFactor;
  poseOnSeg[4] /= rotToTransFactor;

  return onEnd;
}


/*
    d =	(|(x_0-x_1)x(x_0-x_2)|)/(|x_2-x_1|)
*/

//double roPathSegment::GetDistance(vct5 a)
//{

//    double dist = (a - GetClosestPointOnLine(a)).Norm();
//    return dist;
//}


/* A vector along the line is given by
    v= [ x_1+(x_2-x_1)t;
         y_1+(y_2-y_1)t;
         z_1+(z_2-z_1)t
       ].
    t=-((x_1-x_0)Â·(x_2-x_1))/(|x_2-x_1|^2),

    Returns the closest point and also the t value from the start point
*/
//vct5 roPathSegment::GetClosestPointOnLine(vct5 a)
//{
//    double numer = vctDotProduct(a-start, end-start);
//    double denom = (end-start).NormSquare();
//    double t = numer/denom;

//    if(t < 0.0)
//        return start;
//    else if(t > 1.0)
//        return end;

//    return start + (end-start)*t;
//}

//vct5 roPathSegment::GetClosestPointOnLine(vct5 a,  double transToRotFactor){
  
//  double numer = vctDotProduct(a-start, end-start);
//  double denom = (end-start).NormSquare();
//  double t = numer/denom;
  
//  if(t < 0.0)
//    return start;
//  else if(t > 1.0)
//    return end;
  
//  return start + (end-start)*t;
//}

////GetParametrization of the segment (how far along it are we).
//// t is the var name used by RHT
//double roPathSegment::GetNormalizedDistAlongVector(vct5 a)
//{
//    double numer = vctDotProduct(a-start, end-start);
//    double denom = (end-start).NormSquare();

//    return numer/denom;
//}

//
void roPathSegment::Test() {

    vct5 start1(0.0,0.0,0.0,0.0,0.0);
    vct5 end1(1.0, 0.0, 0.0, cmnPI / 6.0, cmnPI / 6.0);
    vct5 testP(2.5, 2.0, 0.0, cmnPI / 6.3, cmnPI / 6.3);
    roPathSegment seg(start1, end1, 0);

    double r2tFact = 100 ;
    double distToPoint = 0.0;
    double distNormT = 0.0;
    vct5 resP;
    bool r = seg.GetClosestPoint(testP, r2tFact, resP,distToPoint, distNormT);

    std::cout << resP << std::endl;
    std::cout << distToPoint << std::endl;
    std::cout << distNormT << std::endl;
    std::cout << r << std::endl;

}

