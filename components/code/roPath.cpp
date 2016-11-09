/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
 $Id: $
 
 Author(s):  Marcin Balicki
 Created on: 2014
 
 (C) Copyright 2006-2014Johns Hopkins University (JHU), All Rights
 Reserved.
 
 --- begin cisst license - do not edit ---
 
 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.
 
 --- end cisst license ---
 
 */

#include <sawConstraintController/roPath.h>
#include <sawConstraintController/roPathSegment.h>
#include <limits>       // std::numeric_limits
#include <cisstOSAbstraction/osaGetTime.h>

CMN_IMPLEMENT_SERVICES(roPath);

roPath::roPath(void)
{
  
  Reset();
}

/// \todo remove FDPoint - not required.
void roPath::GenerateSegmentsFromData(void)
{
  HeadSegments.clear();

  vct5 prevPose, currPose;

  for (unsigned int i = 0; i < HeadPos.size(); ++i)
  {
    currPose = GetXYZRT_FromFrm(Frames[i]);
    while(currPose[3] < cmnPI/2.0)
    {
      currPose[3] += 2.0*cmnPI;
    }

    if(i > 0)
    {
      roPathSegment *L = new roPathSegment(prevPose, currPose, i-1);
      HeadSegments.push_back(L);
    }
    prevPose =  currPose;
  }

  CMN_LOG_CLASS_RUN_WARNING << " Generate5DPoints with " << HeadSegments.size() << " Segments" << std::endl;
}

//vct5 roPath::GetClosestPointOnPath(vct5 testPose)
//{
//  // Closest point on the path of start positions
//  std::vector<roPathSegment *> possibleLines;
//  std::vector<vct5> possiblePoints;
//
//
//  for (int i = 0; i < HeadSegments.size(); ++i)
//  {
//    roPathSegment* L = HeadSegments.at(i);
//
//    // mulitplier
//    vct5 point = L->GetClosestPointOnLine(testPose);
//    
//    possibleLines.push_back(L);
//    possiblePoints.push_back(point);
//  }
//  
//  double closestDist = 1000000;
//  vct5 closestPointToStartPos;
//  roPathSegment *closestLine;
//  
//  if(possibleLines.size() == 0)
//  {
//    std::cerr << "No Closest Lines" << std::endl;
//    exit(1);
//  }
//  
//  // std::cout << startPos << std::endl;
//  for (int i = 0; i < possibleLines.size(); ++i)
//  {
//    double dist = possibleLines.at(i)->GetDistance(testPose);
//    double T = possibleLines.at(i)->GetNormalizedDistAlongVector(testPose);
//    
//    // std::cout << dist << std::endl;
//    
//    if(dist < closestDist || (dist == closestDist && T >= 0 && T < 1))
//    {
//      closestDist = dist;
//      closestLine = possibleLines.at(i);
//      closestPointToStartPos = possiblePoints.at(i);
//    }
//  }
//  
//  return closestPointToStartPos;
//}

vctFrm3 roPath::GetClosestPointOnPath(const vctFrm3 &inFrame, double rotToTransFactor) {
    
  if (HeadSegments.size() < 1) {
    CMN_LOG_CLASS_RUN_ERROR << "NOT ENOUGH SEGMENTS IN PATH" << std::endl;
    return inFrame;
  }

  vct5 inPoseVec = GetXYZRT_FromFrm(inFrame);
    while(inPoseVec[3] < cmnPI/2.0)
    {
      inPoseVec[3] += 2.0*cmnPI;
    }
  
  // Closest point on the path of test positions
  double closestDist = std::numeric_limits<double>::max();
  vct5 closestPoint;
  double distToPoint;
  double distAlongVecToPoint;
  // mulitplier
  vct5 poseOnPath;

  for (unsigned int i = 0; i < HeadSegments.size(); ++i)
  {
    bool r = HeadSegments.at(i)->GetClosestPoint(inPoseVec, rotToTransFactor, poseOnPath, distToPoint, distAlongVecToPoint);
    
    //while(poseOnPath[3] < cmnPI/2.0)
    //{
    //  poseOnPath[3] += 2.0*cmnPI;
    //}

    //if (!r)
    //{
    //  std::cout << " GetClosestPoint: \nRobot Frame: " << inPoseVec << "\nClosest Point: " << poseOnPath << "\nFirst Point: " << HeadSegments.at(0)->start << std::endl;
    //}
    //REVIEW THIS (SEcond part especially)
    if(distToPoint < closestDist ||
       (distToPoint == closestDist && distAlongVecToPoint >= 0 && distAlongVecToPoint < 1))
    {
      closestDist = distToPoint;
      closestPoint = poseOnPath;
    }
  }
  
  vctFrm3 resultFrm = GetFrmFromXYZRT(closestPoint);
  
  std::cout << " inFrame(curr+force)\n "  << inFrame << std::endl;
  std::cout << " resultFrm (path)\n "  << resultFrm << std::endl;
  std::cout << " closest point on path \n " << closestPoint << std::endl;

  return resultFrm;
  
}


//vct5 roPath::GetClosestPointOnPath(vct5 testPose,  double transToRotFactor)
//{
//  // Closest point on the path of test positions
//  
//  std::vector<roPathSegment *> possibleLines;
//  std::vector<vct5> possiblePoints;
//  
//  for (int i = 0; i < HeadSegments.size(); ++i)
//  {
//    roPathSegment* L = HeadSegments.at(i);
//    
//    // mulitplier
//    vct5 point = L->GetClosestPointOnLine(testPose, transToRotFactor);
//    
//    possibleLines.push_back(L);
//    possiblePoints.push_back(point);
//  }
//  
//  double closestDist = 1000000;
//  vct5 closestPointToStartPos;
//  //roPathSegment *closestLine;
//  
//  if(possibleLines.size() == 0)
//  {
//    std::cerr << "No Closest Lines" << std::endl;
//    exit(1);
//  }
//  
//  // std::cout << testPose << std::endl;
//  for (int i = 0; i < possibleLines.size(); ++i)
//  {
//    /// \todo - double check this!!!
//    /// change to (testPose - this point).Norm();
//    double dist = possibleLines.at(i)->GetDistance(testPose);
//    //this can be returned earlier from getclosestPoseOnLine
//    double T = possibleLines.at(i)->GetNormalizedDistAlongVector(testPose);
//    
//    // std::cout << dist << std::endl;
//    //REVIEW THIS (SEcond part especially)
//    if(dist < closestDist || (dist == closestDist && T >= 0 && T < 1))
//    {
//      closestDist = dist;
//      //closestLine = possibleLines.at(i);
//      closestPointToStartPos = possiblePoints.at(i);
//    }
//  }
//  
//  return closestPointToStartPos;
//}


bool roPath::LoadHeadsTails(const std::string fileName) {
  
  std::ifstream myfile;
  myfile.open(fileName.c_str(), std::ifstream::in);
  std::string line;
  vct6 headtail;
  int lineNum = 1;
  
  if(!myfile.is_open())
  {
    CMN_LOG_CLASS_RUN_ERROR << "Cannot Open File " << fileName << std::endl;
    return false;
  }
  
  Reset();
  int i = 0;
  char delim = ',';
  std::string numStr;

  while (std::getline(myfile, numStr, delim))
  {
    std::stringstream lineStream;
    lineStream << numStr;
    lineStream >> headtail[i++];
    if (i == 6) {
        AddHeadTail(headtail);
        lineNum++;
        i = 0;
    }
  }
  
  if(lineNum == 1)
  {
    CMN_LOG_CLASS_RUN_ERROR << "File is Empty" << std::endl;
  }
  std::cout << "Total Number of Points loaded " << HeadPos.size() << std::endl;
  myfile.close();
  
  
  if(HeadPos.size() != TailPos.size())
  {
    CMN_LOG_CLASS_RUN_ERROR << "Size of Start and End location don't match" << std::endl;
    Reset();
    return false;
  }
  
  GenerateSegmentsFromData();
  
  return true;
}


bool roPath::LoadFrames(const std::string fileName){
  
  std::ifstream myfile;
  myfile.open(fileName.c_str(), std::ifstream::in);
  std::string line;
  vct7 xyzWIJK;
  
  int lineNum = 1;
  
  if(!myfile.is_open())
  {
    CMN_LOG_CLASS_RUN_ERROR << "Cannot Open File " << fileName << std::endl;
    return false;
  }
  
  Reset();

  int i = 0;
  char delim = ',';
  std::string numStr;
  while (std::getline(myfile, numStr, delim))
  {
    std::stringstream lineStream;
    lineStream << numStr;
    lineStream >> xyzWIJK[i++];
    if (i == 7) {
        vctFrm3 frame;
        frame.Translation() = xyzWIJK.XYZ();
        frame.Rotation().From(vctQuatRot3(xyzWIJK[4], xyzWIJK[5], xyzWIJK[6], xyzWIJK[3]));
        AddFrame(frame);
        lineNum++;
        i = 0;
    }
  }

  if(lineNum == 1)
  {
    CMN_LOG_CLASS_RUN_ERROR << "File is Empty" << std::endl;
  }
  std::cout << "Total Number of Points loaded " << HeadPos.size() << std::endl;
  myfile.close();
  
  GenerateSegmentsFromData();
  
  return true;
}

bool roPath::LoadTailHeadMatrix(const vctDoubleMat &mat) {
  
  int n = mat.sizes().X();
  if (mat.sizes().Y() != 6  || n == 0) {
    CMN_LOG_CLASS_RUN_ERROR << "matrix is wrong size" << std::endl;
    return false;
  }
  Reset();
  for (int i = 0; i < n; ++i){
    AddHeadTail(vct6(mat[i]));
  }
  CMN_LOG_CLASS_RUN_VERBOSE << "Added " << n << " path samples" << std::endl;  
  GenerateSegmentsFromData();
  return true;
}

bool roPath::SaveFrames(const std::string filePrefix){
  
  std::ofstream logFile;
  
  std::string dateTime;
  osaGetDateTimeString(dateTime);
  std::string fileName;
  
  fileName = filePrefix + dateTime + std::string("XYZWIJK.csv");
  logFile.open(fileName.c_str(), std::ios::out | std::ios::app);
  
  if (!logFile.is_open()) {
    logFile.close();
    CMN_LOG_CLASS_INIT_VERBOSE << "Failed to open file  : " << fileName << std::endl;
    return false;
  }
  
  std::string delim(",");
  
  for (unsigned int i = 0; i < Frames.size(); ++i)
  {
    vctQuaternionRotation3<double> q(Frames[i].Rotation());
    logFile << std::setprecision(10)
    << Frames[i].Translation().X() << delim
    << Frames[i].Translation().Y() << delim
    << Frames[i].Translation().Z() << delim
    << q.W() << delim
    << q.X() << delim
    << q.Y() << delim
    << q.Z() << std::endl;
    
  }
  
  logFile.close();
  
  CMN_LOG_CLASS_INIT_VERBOSE << "Writen " << GetNumOfSamples() << " to file  : " << fileName << std::endl;
  
  return true;
}
bool roPath::SaveHeadsTails(const std::string filePrefix){
  
  std::ofstream  logFile;
  
  std::string dateTime;
  osaGetDateTimeString(dateTime);
  std::string fileName;
  
  fileName = filePrefix + dateTime + std::string("XYZXYZ.csv");
  logFile.open(fileName.c_str(), std::ios::out | std::ios::app);
  
  if (!logFile.is_open()) {
    logFile.close();
    CMN_LOG_CLASS_INIT_VERBOSE << "Failed to open file  : " << fileName << std::endl;
    return false;
  }
  
  std::string delim(",");
  
  for (unsigned int i = 0; i < Frames.size(); ++i)
  {
    logFile << std::setprecision(10)
    << HeadPos[i][0] << delim
    << HeadPos[i][1] << delim
    << HeadPos[i][2] << delim
    << TailPos[i][0] << delim
    << TailPos[i][1] << delim
    << TailPos[i][2] << std::endl;
  }
  
  logFile.close();
  
  CMN_LOG_CLASS_INIT_VERBOSE << "Writen " << GetNumOfSamples() << " to file  : " << fileName << std::endl;
  
  return true;
}
void roPath::Reset(){

  for (unsigned int i = 0; i < HeadSegments.size(); ++i)
  {
      delete HeadSegments[i];
  }

  HeadSegments.clear();
  HeadPos.clear();
  TailPos.clear();
  Frames.clear();
}

unsigned int roPath::GetNumOfSamples(){
  return Frames.size();
}

void roPath::AddFrame(const vctFrm3 &frame) {
  Frames.push_back(frame);
  HeadPos.push_back(frame.Translation());
  vct3 tailOffsetZ(0.0, 0.0, 100);
  TailPos.push_back(frame * tailOffsetZ);
  /// \todo Only do this at the end of the process.
}

void roPath::AddHeadTail(const vct3 &head, const vct3 &tail) {
  HeadPos.push_back(head);
  TailPos.push_back(tail);
  Frames.push_back(FrameFromHeadTail(head, tail));
  /// \todo Only do this at the end of the process.
}

void roPath::AddHeadTail(const vct6 &ht) {
  double new_roll = ht[3];
  //while(new_roll < cmnPI/2.0)
  //{
  //  new_roll += 2.0*cmnPI;
  //}
  HeadPos.push_back(ht.XYZ());
  vct3 tail(new_roll,ht[4],ht[5]);
  TailPos.push_back(tail);
  Frames.push_back(FrameFromHeadTail(ht.XYZ(), tail));
  /// \todo Only do this at the end of the process.
}


void roPath::GetClosestFrames(vct3 point, vctFrm3 &outFrame1, vctFrm3 &outFrame2){
  CMN_LOG_CLASS_RUN_ERROR << "GetClosestFrames not implemented yet" << point << " ; " << outFrame1 << " ; " << outFrame2 << std::endl;
}

bool roPath::GetLastFrame(vctFrm3 &frame) {
  
  if(GetNumOfSamples() > 0) {
    frame = Frames.back();
    return true;
  }
  return false;
}


void roPath::GetHeadTails(vctDoubleMat &m) const {
    m.resize(HeadPos.size(), 6);
    typedef vctDynamicMatrix<double> MatrixType;

    for (unsigned int i = 0; i < HeadPos.size(); ++i)
    {
        for (unsigned int j = 0; j < 3; ++j)
        {
            m(i, j) = HeadPos[i][j];
            m(i, j+3) = TailPos[i][j];

        }
//        MatrixType::Submatrix::Type sub0_2(m, i, 0, 1, 3);
//        sub0_2 = vctDoubleVec(HeadPos[i]);

//        MatrixType::Submatrix::Type sub3_5(m, i, 3, 1, 3);
//        sub3_5.Assign(TailPos[i]);
     }
}

/// \todo add angle comparison. only consider z axis.
/// there is a singularity issue here when the Z is rotated about the other z = constant angle.
bool roPath::IsCloseToLastSample(const vctFrm3 &frame, const double &posTolmm, const double &angToldeg) {
  
  vctFrm3 frmIn(frame);
  vctFrm3 frmLast;
  
  //first sample
  if (!GetLastFrame(frmLast))
    return false;
  
  vct3    zAxis1;
  zAxis1[0] = frame.Rotation().at(0,2);
  zAxis1[1] = frame.Rotation().at(1,2);
  zAxis1[2] = frame.Rotation().at(2,2);
  
  vct3    zAxis2;
  zAxis2[0] = frmLast.Rotation().at(0,2);
  zAxis2[1] = frmLast.Rotation().at(1,2);
  zAxis2[2] = frmLast.Rotation().at(2,2);
  
  double a = AngleBetweenVectors(zAxis1, zAxis2);
  
  //check if close
  vct3 diff = frmLast.Translation() - frmIn.Translation();
  
  //is close
  if (diff.Norm() <  posTolmm &&  a < angToldeg)
    return true;
  else //far
    return false;
}

//returns angle in degrees.
double roPath::AngleBetweenVectors(const vct3& v1, const vct3 & v2) {
  
  vct3 v1n = v1.Normalized();
  vct3 v2n = v2.Normalized();
  double dp = vctDotProduct(v1n, v2n);
  return acos(dp) * cmn180_PI;

}

vctFrm3 roPath::FrameFromHeadTail(const vct3 &head, const vct3 &tail) {
  
  vctFrm3 frame;
  
  //z normalize
  vct3 zn = (tail - head).Normalized();
  vct3 x(1.0,0.0,0.0);
  
  // check that they are no colinear
  if (zn[0] == x[0]) {
    CMN_LOG_CLASS_RUN_WARNING << "Need to choose another x vector for FrameFromHeadTail" << std::endl;
    x = vct3(0.0, 1.0, 0.0);
  }
  vct3 yn;
  yn.CrossProductOf(zn, x);
  yn.NormalizedSelf();
  vct3 xn;
  xn.CrossProductOf(yn, zn); // orthogonal unit vectors so xn is also unit
  frame.Rotation().Column(0) = xn;
  frame.Rotation().Column(1) = yn;
  frame.Rotation().Column(2) = zn;
  
  frame.Translation() = head;
  return frame;
  
}

//
vct5 roPath::GetXYZRT_FromFrm(const vctFrm3 & frm) {

    vct3 zN = frm.Rotation().Column(2);
    vct3 projectRoll(0.0, zN.Y(), zN.Z());

    //calculate the roll angle from the rotation matrix
    projectRoll.NormalizedSelf(); //just in case
    double rollAngle = atan2(-projectRoll.Y(), projectRoll.Z());

    //x axis rotated about roll angle (undo so negative)
    vctRot3 r(vctAxAnRot3(vct3(1.0, 0.0, 0.0), -rollAngle));
    ///tilt with roll undon
    vct3 projectedTilt = r * zN;

    double tiltAngle = atan2(projectedTilt.X(),projectedTilt.Z());

    vct5 inPoseVec;
    inPoseVec.XYZ() = frm.Translation();
    inPoseVec[3] = rollAngle;
    inPoseVec[4] = tiltAngle;

    return inPoseVec;
}

// xyz roll tilt
vctFrm3 roPath::GetFrmFromXYZRT(const vct5 & xyzrt ) {

    vctFrm3 resultFrm;
    resultFrm.Translation()= xyzrt.XYZ();

    vctRot3 Rx(vctAxAnRot3(vct3(1.0, 0.0, 0.0), xyzrt[3]));
    vctRot3 Ry(vctAxAnRot3(vct3(0.0, 1.0, 0.0), xyzrt[4]));

    resultFrm.Rotation() = Rx * Ry;
    return resultFrm;
}

void roPath::Test() {

  vctRot3 Rx(vctAxAnRot3(vct3(1.0, 0.0, 0.0), 10 * cmnPI_180 ));
  vctRot3 Ry(vctAxAnRot3(vct3(0.0, 1.0, 0.0), -15 * cmnPI_180));
  
  vctRot3 Rxy(Rx * Ry);
  
  vctFrm3 F;
  F.Translation() = vct3(20,34,56);
  F.Rotation() = Rxy;

  vct5 v = GetXYZRT_FromFrm(F);
  vctFrm3 F2 = GetFrmFromXYZRT(v);

  std::cout << F  << std::endl;
  std::cout << F2  << std::endl;


}


