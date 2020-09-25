#ifndef MTSVFMESH_H
#define MTSVFMESH_H

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <sawConstraintController/mtsVFBase.h>
#include <sawConstraintController/mtsVFCartVel.h>
#include <sawConstraintController/mtsVFDataMesh.h>

#include <cisstMesh/msh3PDTreeMesh.h>
#include <cisstMesh/msh3AlgPDTreeCPMesh.h>
#include <cisstMesh/msh3Mesh.h>

class CISST_EXPORT mtsVFMesh : public mtsVFCartesianTranslation
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_VERBOSE);
public:
    mtsVFMesh();

    /*! Constructor
    \param name String name of object
    */
    mtsVFMesh(const std::string & name, mtsVFDataBase * data, msh3Mesh & mesh);

    //! Updates co with virtual fixture data.
    /*! FillInTableauRefs
    */
    void FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime) override;
    void ComputeConstraintSize() override;

    void ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double tickTime) override;

    void TransformMesh(const vctFrm4x4 & transformation, msh3Mesh & mesh);
    void ConstructPDTree(msh3Mesh & mesh);

protected:
    prmKinematicsState * CurrentKinematics;
    msh3PDTreeMesh* pTreeMesh;
    msh3AlgPDTreeCPMesh* pAlgMesh;
    vct3 mCurrentPosition;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsVFMesh);

#endif // MTSVFMESH_H
