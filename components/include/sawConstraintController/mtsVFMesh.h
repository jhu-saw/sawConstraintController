#ifndef MTSVFMESH_H
#define MTSVFMESH_H

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFCartVel.h>
#include <sawConstraintController/mtsVFDataMesh.h>

#include <PDTree_Mesh.h>
#include <algPDTree_CP_Mesh.h>
#include <cisstMesh.h>

class CISST_EXPORT mtsVFMesh : public mtsVFCartesianTranslation
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);
public:
    mtsVFMesh();

    /*! Constructor
    \param name String name of object
    */
    mtsVFMesh(const std::string & name, mtsVFDataBase * data, cisstMesh & mesh);

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) override;
    void ComputeConstraintSize() override;

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime) override;

    void TransformMesh(const vctFrm4x4 & transformation, cisstMesh & mesh);
    void ConstructPDTree(cisstMesh& mesh);

protected:
    prmKinematicsState * CurrentKinematics;
    PDTree_Mesh* pTreeMesh;
    algPDTree_CP_Mesh* pAlgMesh;
    vct3 mCurrentPosition;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFMesh);

#endif // MTSVFMESH_H
