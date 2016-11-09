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

#ifndef _roPath_h
#define _roPath_h

#include <iostream>
#include <sstream>
#include <fstream>
#include <cisstVector/vctFrame4x4.h>
#include <cisstVector/vctTypes.h>

class roPathSegment;

/// to do - convert head and tail to frame.
class roPath : public cmnGenericObject
{
  CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);
  
public:
  roPath(void);
  ~roPath(void) {};
  
  // files are SPACE delimited and have CSV suffix.
  /// loads path as frames in XYZ quaternion W IJK
  bool LoadFrames(const std::string fileName);
  /// loads path as head and tail of the pose  tx ty tz hx hy hz
  bool LoadHeadsTails(const std::string fileName);
  
  /// Nx6    (headxyz tailxyz)
  bool LoadTailHeadMatrix(const vctDoubleMat & mat);
  
  ///  path as frames in XYZ quaternion W IJK
  bool SaveFrames(const std::string filePrefix);
  ///  save path as head and tail of the pose  tx ty tz hx hy hz
  bool SaveHeadsTails(const std::string filePrefix);
  
  void Reset();
  /// number of the path points
  unsigned int GetNumOfSamples();
  /// Automatically adds head and tail based on frame
  void AddFrame(const vctFrm3 &frame);
  
  /// Automatically adds frame based on head/tail
  void AddHeadTail(const vct3 &head, const vct3 &tail);
  ///
  void AddHeadTail(const vct6 &ht);

  const std::vector<vctFrm3> & GetFrames() {
    return Frames;
  }
  /// Nx6 list of heads and tails
  void GetHeadTails(vctDoubleMat &m) const;
  
  /// underconstrained def of the path, so the vctFrm is only close to Z unit vec rot and full translation.
  /// rotToTransFactor is the wight between rot and translation (100 * R = t )
  vctFrm3 GetClosestPointOnPath(const vctFrm3 &inPose, double rotToTransFactor);
  
  bool GetLastFrame(vctFrm3 &frame);

  void GenerateSegmentsFromData(void);
  
  //is empty?
  bool Empty() {
    return !GetNumOfSamples();
  }
  
  /// check if we have moved very much?
  /// consider z axis for rotation.
  /// both ang and pos have to be close...
  bool IsCloseToLastSample(const vctFrm3 &frame, const double &posTolmm, const double &angToldeg);
 
  static void Test();
  
protected:
  
  //Uses the 2 norm
  //transToRotFactor adds a scalling to the rot component in calculating the distance
  //multiples input rot and line rot.
  
  vct5 GetClosestPointOnPath(vct5 testPose, double transToRotFactor = 1.0);
  
  void GetClosestFrames(const vct3 point, vctFrm3 &bound1, vctFrm3& bound2);
  
  std::vector<vctFrm3> Frames;
  
  std::vector<vct3>   HeadPos;
  std::vector<vct3>   TailPos;
  std::vector<roPathSegment*>  HeadSegments;
  
  vctFrm3 FrameFromHeadTail(const vct3 &head, const vct3 &tail);
  
  //returns angle in degrees.
  double AngleBetweenVectors(const vct3& v1, const vct3 & v2);


  static vct5 GetXYZRT_FromFrm(const vctFrm3 & frm);
  static vctFrm3 GetFrmFromXYZRT(const vct5 & xyzrt );
  
};


CMN_DECLARE_SERVICES_INSTANTIATION(roPath);

#endif
