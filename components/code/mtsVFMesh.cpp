#include <sawConstraintController/mtsVFMesh.h>

CMN_IMPLEMENT_SERVICES(mtsVFMesh)

mtsVFMesh::mtsVFMesh() : mtsVFCartesianTranslation(){}


mtsVFMesh::mtsVFMesh(const std::string &name, mtsVFDataBase *data, cisstMesh &mesh) : mtsVFCartesianTranslation(name,data) {
    int nThresh = 5; double diagThresh = 5.0;
    // construct PD-Tree
    pTreeMesh = new PDTree_Mesh(mesh, nThresh, diagThresh);
    pAlgMesh = new algPDTree_CP_Mesh(pTreeMesh);
    pTreeMesh->SetSearchAlgorithm(pAlgMesh);
}

void mtsVFMesh::FillInTableauRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{
    // This constraint only works with position (no rotation)

    // get the data
    mtsVFDataMesh* meshData = reinterpret_cast<mtsVFDataMesh*>(Data);

    // if there is inequality constraint
    if (meshData->IneqConstraintRows > 0) {
        vct3 CurrentPos(CurrentKinematics->Frame.Translation()); // ignore rotation for now
        std::cout << "Fill in Tableau for mesh" << std::endl;
        std::cout << "Active constraint " << meshData->IneqConstraintRows << std::endl;
        int rowNumber = 0;
        vctDoubleVec N; N.SetSize(3);
        vctDoubleVec NJ; NJ.SetSize(6);
        vctDoubleMat JacPos(CurrentKinematics->Jacobian.Ref(3,meshData->NumJoints,0,0));
        for (auto it : meshData->ActiveFaceIdx){
            // get normal direction
            N.Assign(pTreeMesh->Mesh->activeNormal.at(it)).NormalizedSelf();
            IneqConstraintVectorRef.at(rowNumber) = - (CurrentPos-pTreeMesh->Mesh->closestPoint.at(it)).DotProduct(vct3(N));

            std::cout << "Face index " << it << " Normal " << N << " Distance " << IneqConstraintVectorRef.at(rowNumber) << std::endl;

            if (mode == mtsVFBase::CONTROLLERMODE::JPOS || mode == mtsVFBase::CONTROLLERMODE::JVEL){
                NJ.ProductOf(N,JacPos);
                IneqConstraintMatrixRef.Row(rowNumber).Assign(NJ); //TODO: why is direct ProductOf(N,JacPos) not working???
            }
            else if (mode == mtsVFBase::CONTROLLERMODE::CARTPOS || mode == mtsVFBase::CONTROLLERMODE::CARTVEL){
                IneqConstraintMatrixRef.Row(rowNumber).Assign(N);
            }
            rowNumber ++;
        }
    }
}

void mtsVFMesh::ComputeConstraintSize()
{
    if(Kinematics.size() < 1)
    {
        CMN_LOG_CLASS_RUN_ERROR << "ComputeRequiredConstraint: Mesh VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Mesh VF given improper input");
    }

    // Pointer to kinematics
    CurrentKinematics = Kinematics.at(0);
    vct3 CurrentPos(CurrentKinematics->Frame.Translation()); // ignore rotation for now

    // get the data
    mtsVFDataMesh* meshData = reinterpret_cast<mtsVFDataMesh*>(Data);

    if(!meshData)
    {
        CMN_LOG_CLASS_RUN_ERROR << "Mesh data object not set" << std::endl;
        return;
    }

    // find closest point
    pTreeMesh->Mesh->ResetMeshConstraintValues();
    meshData->ActiveFaceIdx.clear();
    meshData->IneqConstraintRows = 0;
    std::cout << "\nFind intersection" << std::endl;
    int numIntersected = pTreeMesh->FindIntersectedPoints(CurrentPos,meshData->BoudingDistance,meshData->ActiveFaceIdx);

    // if there is intersection
    if (numIntersected > 0){
        // merge edge points
        pTreeMesh->Mesh->MergeEdgePoint(meshData->ActiveFaceIdx, CurrentPos);
        meshData->IneqConstraintRows = meshData->ActiveFaceIdx.size();
    }

}

void mtsVFMesh::ConvertRefs(const mtsVFBase::CONTROLLERMODE mode, const double TickTime)
{

}
