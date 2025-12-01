#ifndef WORKDATA_H
#define WORKDATA_H

#include "../Estimator/IncrementalMapper.h"
#include "../Feature/Bitmap.h"
#include "../Feature/FeatureExtractor.h"
#include "../Feature/FeatureMatcher.h"
#include "../Feature/ImageReader.h"
#include "../Scene/Database.h"
#include "../Scene/DatabaseCache.h"
#include "../Scene/ReconstructionManager.h"
#include "../Scene/RenderData.h"
#include "tbb/concurrent_map.h"
#include <QMutex>
#include <QMutexLocker>
#include <QObject>
#include <QReadWriteLock>
#include <QSemaphore>
#include <QThread>

enum CImageStatusFlag : char {
	CUnread = 0,
	CRead = 1,
	CExtracting = 2,
	CExtracted = 3,
	CMatching = 4,
	CMatched = 5,
	CTwoViewGeometryEstimating = 6,
	CTwoViewGeometryEstimated = 7
};

// 延迟锁
class DelayedWriteLocker {
public:
	DelayedWriteLocker(QReadWriteLock& lock, int maxRetries = 3, int delayMs = 100)
		: m_lock(lock)
	{
		for (int i = 0; i < maxRetries; ++i) {
			if (m_lock.tryLockForWrite()) return;
			QThread::msleep(delayMs);
		}
		m_lock.lockForWrite(); // 最后阻塞
	}

	~DelayedWriteLocker() { m_lock.unlock(); }

private:
	QReadWriteLock& m_lock;
};


namespace {
	void ScaleKeypoints(const Bitmap& bitmap, const Camera& camera, FeatureKeypoints* keypoints) {
		if (static_cast<size_t>(bitmap.Width()) != camera.Width() ||
			static_cast<size_t>(bitmap.Height()) != camera.Height()) {

			const float scale_x = static_cast<float>(camera.Width()) / bitmap.Width();
			const float scale_y = static_cast<float>(camera.Height()) / bitmap.Height();

			for (auto& keypoint : *keypoints) {
				keypoint.Rescale(scale_x, scale_y);
			}
		}
	}
	size_t CountFalseMoreThanTrue(const tbb::concurrent_map<size_t, std::atomic_bool>& p_estimatedImageIDs) {
		size_t false_count = 0;
		size_t true_count = 0;
		for (const auto& kv : p_estimatedImageIDs) {
			if (kv.second.load()) {
				++true_count;
			}
			else {
				++false_count;
			}
		}
		return false_count - true_count;
	}
	size_t CountFalse(const tbb::concurrent_map<size_t, std::atomic_bool>& p_estimatedImageIDs) {
		size_t false_count = 0;
		for (const auto& kv : p_estimatedImageIDs) {
			if (!kv.second.load()) {
				++false_count;
			}
		}
		return false_count;
	}
}

class WorkData : public QObject {
	Q_OBJECT

public:
	explicit WorkData(QObject* parent = nullptr)
		: QObject(parent),
		database(nullptr),
		database_cache(nullptr),
		modelManager(nullptr),
		mapper(nullptr),
		globalFeatureExtractor(nullptr),
		globalFeatureRetriever(nullptr) {}

	void Initial(Database* p_database, ReconstructionManager* p_modelManager) {
		database = p_database;
		modelManager = p_modelManager;
	}

	//// 对外提供公共资源操作方法
	// set/get
	Database* getDatabase() const { return database; }
	std::shared_ptr<const DatabaseCache> getDatabaseCache() const { return database_cache; }
	ReconstructionManager* getModelManager() const { return modelManager; }
	ReconstructionManager* getModelManager() { return modelManager; }
	IncrementalMapper* getIncrementalMapper() { return mapper; }
	std::set<size_t> getImageIDsMask() { return imageIDsMask; }
	CGlobalFeatureExtractor* getGlobalFeatureExtractor() const { return globalFeatureExtractor; }
	CGlobalFeatureRetriever* getGlobalFeatureRetriever() const { return globalFeatureRetriever; }
	void setGlobalFeatureFunc(CGlobalFeatureExtractor* p_globalFeatureExtractor,
		CGlobalFeatureRetriever* p_globalFeatureRetriever) {
		globalFeatureExtractor = p_globalFeatureExtractor;
		globalFeatureRetriever = p_globalFeatureRetriever;
	}


	//// === Database 相关操作 ===
	camera_t DatabaseWriteCamera(Camera camera) {
		QWriteLocker locker(&m_database_lock);
		return database->WriteCamera(camera);
	}

	image_t DatabaseWriteImage(Image image) {
		QWriteLocker locker(&m_database_lock);
		return database->WriteImage(image);
	}

	size_t DatabaseReadImageNum() {
		QReadLocker locker(&m_database_lock);
		return database->NumImages();
	}

	void DatabaseWriteKeypoints(image_t imageid, FeatureKeypoints& p_keypoints) {
		QWriteLocker locker(&m_database_lock);
		database->WriteKeypoints(imageid, p_keypoints);
	}

	void DatabaseWriteDescriptors(image_t imageid, FeatureDescriptors& p_descriptors) {
		QWriteLocker locker(&m_database_lock);
		database->WriteDescriptors(imageid, p_descriptors);
	}

	Camera DatabaseReadCamera(camera_t cameraid) {
		QReadLocker locker(&m_database_lock);
		return database->ReadCamera(cameraid);
	}

	Image DatabaseReadImage(image_t imageid) {
		QReadLocker locker(&m_database_lock);
		return database->ReadImage(imageid);
	}

	void DatabaseWriteGlobalFeature(image_t imageID, GlobalFeature& globalFeature) {
		QWriteLocker locker(&m_database_lock);
		database->WriteGlobalFeature(imageID, globalFeature);
	}

	FeatureDescriptors DatabaseReadDescriptors(image_t imageID) {
		QReadLocker locker(&m_database_lock);
		return database->ReadDescriptors(imageID);
	}

	void DatabaseWriteMatches(image_t id1, image_t id2, FeatureMatches& matches) {
		QWriteLocker locker(&m_database_lock);
		database->WriteMatches(id1, id2, matches);
	}

	bool DatabaseExistsMatches(image_t id1, image_t id2) {
		QReadLocker locker(&m_database_lock);
		return database->ExistsMatches(id1, id2);
	}

	FeatureMatches DatabaseReadMatches(image_t id1, image_t id2) {
		QReadLocker locker(&m_database_lock);
		return database->ReadMatches(id1, id2);
	}

	FeatureKeypoints DatabaseReadKeypoints(image_t imageid) {
		QReadLocker locker(&m_database_lock);
		return database->ReadKeypoints(imageid);
	}

	void DatabaseWriteTwoViewGeometry(image_t id1, image_t id2, const TwoViewGeometry& twoviewgeo) {
		QWriteLocker locker(&m_database_lock);
		database->WriteTwoViewGeometry(id1, id2, twoviewgeo);
	}

	bool DatabaseExistsInlierMatches(image_t id1, image_t id2) {
		QReadLocker locker(&m_database_lock);
		return database->ExistsInlierMatches(id1, id2);
	}

	TwoViewGeometry DatabaseReadTwoViewGeometry(image_t id1, image_t id2) {
		QReadLocker locker(&m_database_lock);
		return database->ReadTwoViewGeometry(id1, id2);
	}

	void ReadTwoViewGeometryNumInliers(std::vector<std::pair<image_t, image_t>>& imgPairs, std::vector<int>& numInliers) {
		QReadLocker locker(&m_database_lock);
		database->ReadTwoViewGeometryNumInliers(&imgPairs, &numInliers);
	}

	//// === ModelManager 相关操作 ===
	size_t ModelManagerGetNum() {
		QReadLocker locker(&m_modelmanager_lock);
		return modelManager->Size();
	}

	size_t ModelManagerAddModel() {
		QWriteLocker locker(&m_modelmanager_lock);
		return modelManager->Add();
	}

	//// === DatabaseCache 相关操作 ===
	std::set<size_t> getImageMask(const int length, bool contain = true) {
		QReadLocker locker(&m_reconstructedSet_lock);
		if (reconstructedImages.empty()) {
			return {};
		}

		size_t maxReconstructed = *reconstructedImages.rbegin();
		auto startIt = estimatedImages.find(maxReconstructed);
		if (startIt == estimatedImages.end()) {
			return {};
		}

		auto endIt = startIt;
		++endIt;
		for (int i = 0; i < length && endIt != estimatedImages.end(); ++i) {
			++endIt;
		}

		if (contain) {
			return { estimatedImages.begin(), endIt };
		}
		else {
			auto fromIt = startIt;
			++fromIt;
			return { fromIt, endIt };
		}
	}

	void LoadCache(const int bufferSize) {
		if (!database_cache) {
			database_cache = std::make_shared<DatabaseCache>();
			database_cache->Create(*database, 10, true, {});
			if (mapper) delete mapper;
			mapper = new IncrementalMapper(database_cache);
			return;
		}
		imageIDsMask = getImageMask(bufferSize);
		database_cache = std::make_shared<DatabaseCache>();
		database_cache->Create(*database, 10, true, imageIDsMask);
		if (mapper) delete mapper;
		mapper = new IncrementalMapper(database_cache);
	}

	bool CheckCache(size_t imageID) {
		if (!database_cache) {
			return false;
		}
		return database_cache->ExistsImage(imageID);
	}

	//// === Image 相关操作 ===
	image_t getImageID(const std::string imagePath) {
		QReadLocker locker(&m_imageidpath_lock);
		return allImagePaths.at(imagePath);
	}

	std::string getImagePath(const image_t imageID) {
		QReadLocker locker(&m_imageidpath_lock);
		return allImages.at(imageID);
	}

	void addImageMap(image_t imageid, std::string imagepath) {
		QWriteLocker locker(&m_imageidpath_lock);
		allImages[imageid] = imagepath;
		allImagePaths[imagepath] = imageid;
	}

	void setImageStatus(std::string imagepath, CImageStatusFlag status) {
		QMutexLocker locker(&m_imagestatus_lock);
		imagesStatus[imagepath] = status;
	}

	//// === Bitmap 相关操作 ===
	void WriteBitmap(Bitmap& p_bitmap, image_t imageid) {
		QWriteLocker locker(&m_bitmap_lock);
		allBitmaps[imageid] = p_bitmap;
	}

	Bitmap ReadBitmap(image_t imageid) {
		QReadLocker locker(&m_bitmap_lock);
		return allBitmaps.at(imageid);
	}

	void ReleaseBitmap(image_t imageid) {
		QWriteLocker locker(&m_bitmap_lock);
		allBitmaps.erase(imageid);
	}

	//// === 模型锁相关操作 ===
	size_t getMaxModelCount() const {
		return m_models_lock.size();
	}


	//// === 其他变量 ===
	void AddEstimatedImage(size_t image_id) {
		QWriteLocker locker(&m_estimatedSet_lock);
		estimatedImages.insert(image_id);
		estimatedImagesNum.store(estimatedImages.size());
	}

	void AddReconstructedImage(size_t image_id) {
		QWriteLocker locker(&m_reconstructedSet_lock);
		reconstructedImages.insert(image_id);
		reconstructImagesNum.store(reconstructedImages.size());
	}

public:
	//// 并发线程计时器
	tbb::concurrent_map<std::size_t, std::atomic_size_t> SIFTExtractionTimes;
	tbb::concurrent_map<std::size_t, std::atomic_size_t> globalFeatureExtractionTimes;
	tbb::concurrent_map<std::size_t, std::atomic_size_t> globalFeatureRetrievalTimes;
	tbb::concurrent_map<std::size_t, std::atomic_size_t> SIFTMatchingTimes;
	tbb::concurrent_map<std::size_t, std::atomic_size_t> geometricVerificationTimes;
	//// 重建计数器
	tbb::concurrent_unordered_map<size_t, std::atomic_size_t> numMatchingPairs;
	std::atomic_size_t readImagesNum{ 0 };
	std::atomic_size_t reconstructImagesNum{ 0 };
	std::atomic_size_t estimatedImagesNum{ 0 };
	//// 用于UI显示的数据
	std::shared_ptr<const RenderData> render_data; // 反馈数据
	QSemaphore freeFrame{ 1 }; // 产生空闲帧数据
	QSemaphore usedFrame{ 0 }; // 渲染已有帧数据

	//// 互斥锁
	mutable QReadWriteLock m_database_lock;
	mutable QReadWriteLock m_modelmanager_lock;
	mutable std::array<QReadWriteLock, 10> m_models_lock;
	mutable QReadWriteLock m_imageidpath_lock;
	mutable QReadWriteLock m_bitmap_lock;
	QMutex m_imagestatus_lock;
	mutable QReadWriteLock m_estimatedSet_lock;
	mutable QReadWriteLock m_reconstructedSet_lock;

private:
	//// 工作变量
	Database* database;
	std::shared_ptr<DatabaseCache> database_cache;
	ReconstructionManager* modelManager;
	IncrementalMapper* mapper;
	CGlobalFeatureExtractor* globalFeatureExtractor;
	CGlobalFeatureRetriever* globalFeatureRetriever;

	std::unordered_map<std::size_t, std::string> allImages;
	std::unordered_map<std::string, std::size_t> allImagePaths;
	std::unordered_map<std::string, CImageStatusFlag> imagesStatus;
	std::unordered_map<std::size_t, Bitmap> allBitmaps;
	std::set<size_t> estimatedImages;
	std::set<size_t> reconstructedImages;
	std::set<size_t> imageIDsMask;  // 当前缓冲区中的imageID掩膜
};

#endif // WORKDATA_H
