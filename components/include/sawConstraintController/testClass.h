#ifndef TESTCLASS_H
#define TESTCLASS_H

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFCartVel.h>
#include <sawConstraintController/mtsVFDataMesh.h>

#include <PDTree_Mesh.h>
#include <algPDTree_CP_Mesh.h>
#include <cisstMesh.h>

class testClass : public mtsVFCartesianTranslation
{
public:
    testClass();
    testClass(const std::string & name, mtsVFDataBase * data);
};

#endif // TESTCLASS_H
