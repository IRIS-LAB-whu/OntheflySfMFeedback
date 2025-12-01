#include "../Base/timer.h"
#include "RetrieveTask.h"
#include "WorkData.h"

void RetrieveTask::run()
{
	std::string imagePathstr = imagePath.toStdString();
	Timer retrievalTimer;
	retrievalTimer.Start();
	const image_t imageID = m_data->getImageID(imagePathstr);
	CGlobalFeatureRetriever* globalFeatureRetriever = m_data->getGlobalFeatureRetriever();
	const std::vector<image_t> retrievalResult = globalFeatureRetriever->Retrieve(imageID);
	m_data->numMatchingPairs[imageID] = retrievalResult.size();

	const size_t retrievalTime = retrievalTimer.ElapsedMicroSeconds() / 1000;
	m_data->globalFeatureRetrievalTimes.at(imageID) = retrievalTime;

	std::string output = "[Retrieval] Result: " + m_data->DatabaseReadImage(imageID).Name() + " -> ";
	for (image_t i = 0; i < retrievalResult.size(); i++)
	{
		emit retrieveTaskFinished(retrievalResult[i], imageID);

		output += m_data->DatabaseReadImage(retrievalResult[i]).Name();
		if (static_cast<size_t>(i) + 2 <= retrievalResult.size())
		{
			output += ", ";
		}
	}
	LOG(INFO) << output;


	if (m_data->numMatchingPairs.find(imageID) != m_data->numMatchingPairs.end() && m_data->numMatchingPairs.at(imageID) > 0)
	{
		m_data->setImageStatus(imagePathstr, CImageStatusFlag::CMatching);
	}
	else
	{
		m_data->setImageStatus(imagePathstr, CImageStatusFlag::CMatched);
	}
}
