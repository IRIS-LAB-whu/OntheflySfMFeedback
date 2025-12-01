#include "../Scene/meshing.h"
#include "../Scene/ReconstructionManager.h"
#include "FeedbackTask.h"
#include "WorkData.h"

void FeedbackTask::run()
{
	std::shared_ptr<MeshManager> meshManager = std::make_shared<MeshManager>();
	ReconstructionManager* manager = m_data->getModelManager();
	{
		QReadLocker locker(&(m_data->m_models_lock[0]));
		meshManager->CopyFromReconstruction(*(manager->Get(0)));
	}
	meshManager->CreateMesh();
	meshManager->GenerateRenderMeshData();
	m_data->render_data = meshManager->GetRenderingData();

	emit taskFinished();
}
