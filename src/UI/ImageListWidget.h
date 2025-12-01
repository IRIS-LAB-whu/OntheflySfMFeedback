#pragma once
#include "../Scene/Database.h"
#include <mutex>       
#include <QCheckBox>
#include <QDockWidget>
#include <QImageReader>
#include <QLayout>    
#include <QListWidget>
#include <QScrollbar>

class ImageListWidget : public QListWidget
{
	Q_OBJECT
public:
	ImageListWidget(QDockWidget* parent, Database* database);
	~ImageListWidget();

	void AddImage(const std::string& imagePath);
	void ClearImage();
	void SetAutoScrollEnabled(bool enabled);
	bool IsAutoScrollEnabled() const;

public slots:
	void itemDoubleClicked_SLOT(QListWidgetItem* item);

private:
	QDockWidget* m_dockParent;
	Database* m_database;
	std::vector<QListWidgetItem*> m_items;
	std::vector<std::string> m_imagePaths;
	std::mutex m_imageListMutex;
	std::atomic_size_t m_changeCount;
	bool m_autoScrollEnabled;
	QCheckBox* m_autoScrollCheckBox;

	QSize CalculateImageSize(QSize& OriginSize);

private slots:
	void onAutoScrollStateChanged(int state);
};

