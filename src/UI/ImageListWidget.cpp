#include "../Base/macros.h"
#include "../Base/misc.h"
#include "ImageListWidget.h"
#include "ImageViewer.h"
#include <iostream>
using namespace std;

namespace {
	//QString转string
	std::string QString2StdString(const QString& qstr)
	{
		return std::string(qstr.toLocal8Bit());
	}

	//string转QString
	QString StdString2QString(const std::string& str)
	{
		return QString::fromLocal8Bit(str.data());
	}
}

ImageListWidget::ImageListWidget(QDockWidget* parent, Database* database)
	: QListWidget(parent)
	, m_dockParent(parent)
	, m_database(database)
	, m_autoScrollEnabled(true)
	, m_changeCount(0)
	, m_autoScrollCheckBox(nullptr)
{
	m_items.clear();
	m_imagePaths.clear();

	m_autoScrollCheckBox = new QCheckBox(tr("Auto Scroll"));
	m_autoScrollCheckBox->setChecked(true);
	connect(m_autoScrollCheckBox, &QCheckBox::stateChanged, this, &ImageListWidget::onAutoScrollStateChanged);

	QWidget* container = new QWidget();
	QVBoxLayout* layout = new QVBoxLayout(container);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);
	layout->addWidget(m_autoScrollCheckBox);
	layout->addWidget(this);
	parent->setWidget(container);
	parent->setWindowTitle(tr("Image list"));

	setSelectionMode(QAbstractItemView::SingleSelection);
	setViewMode(QListWidget::IconMode);
	setIconSize(QSize(180, 180));
	setSpacing(2);
	setResizeMode(QListView::Adjust);
	setMovement(QListView::Static);
	connect(this, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this, SLOT(itemDoubleClicked_SLOT(QListWidgetItem*)));
}

ImageListWidget::~ImageListWidget()
{
	ClearImage();
}

void ImageListWidget::AddImage(const std::string& imagePath)
{
	lock_guard<mutex> lock(m_imageListMutex);
	try
	{
		QScrollBar* scrollbar = verticalScrollBar();
		int currentValue = scrollbar ? scrollbar->value() : 0;
		int maxValue = scrollbar ? scrollbar->maximum() : 0;

		QString QImgPath = QString::fromLocal8Bit(imagePath.data());

		//为了加快读取速度并且降低内存占用, 影像预览并不读取原图, 而是使用ImageReader以图标(Icon)的形式直接读取设定大小的影像图标
		QImageReader ImageReader(QImgPath);
		ImageReader.setAutoTransform(true);
		QSize OriginSize = ImageReader.size();
		QSize TargetSize = OriginSize.scaled(CalculateImageSize(OriginSize), Qt::KeepAspectRatio);
		ImageReader.setScaledSize(TargetSize);
		if (!ImageReader.canRead()) //影像读取失败, 影像可能是损坏的
		{
			cout << "[Warning] Image [ " + imagePath + " ] is damaged!" << endl;
			return;
		}
		QListWidgetItem* NewItem = new QListWidgetItem();
		NewItem->setIcon(QIcon(QPixmap::fromImageReader(&ImageReader)));
		NewItem->setText(StdString2QString(GetFileName(imagePath)));
		TargetSize.setHeight(TargetSize.height() + 10);
		NewItem->setSizeHint(TargetSize);
		NewItem->setData(Qt::UserRole, StdString2QString(GetFileName(imagePath)));
		addItem(NewItem);
		m_items.push_back(NewItem);
		m_imagePaths.push_back(imagePath);
		m_dockParent->setWindowTitle(tr("Image list ") + "(" + QString::number(m_items.size()) + ")");

		if (m_autoScrollEnabled)
		{
			scrollToBottom();
		}

#ifdef DEMO_MODE
		m_changeCount++;
		if (count() > 2 && verticalScrollBar())
		{
			if (m_changeCount >= 2)
			{
				setCurrentRow(count() - 1);
				m_changeCount = 0;
			}
		}
#endif
	}
	catch (const std::exception& e)
	{
		cout << "listwidget error!" << endl;
	}
}

void ImageListWidget::ClearImage()
{
	clear();
	m_items.clear();
	m_imagePaths.clear();
}

void ImageListWidget::itemDoubleClicked_SLOT(QListWidgetItem* item)
{
	//弹出"影像浏览"窗口
	QListWidgetItem* currentItem = this->currentItem();
	if (currentItem)
	{
		size_t index = this->row(currentItem);
		CImageViewer* ImageViewer = new CImageViewer(this, m_database, m_imagePaths[index]);
	}
}

QSize ImageListWidget::CalculateImageSize(QSize& OriginSize)
{
	//计算影像图标合适的大小
	float dWidth = OriginSize.width() / 200.0;
	float dHeight = OriginSize.height() / 200.0;
	float Shrink = max(dHeight, dWidth);
	QSize dsize(OriginSize.width() / Shrink, OriginSize.height() / Shrink);
	return dsize;
}

void ImageListWidget::onAutoScrollStateChanged(int state)
{
	m_autoScrollEnabled = (state == Qt::Checked);
}

bool ImageListWidget::IsAutoScrollEnabled() const
{
	return m_autoScrollEnabled;
}

void ImageListWidget::SetAutoScrollEnabled(bool enabled)
{
	m_autoScrollEnabled = enabled;
	m_autoScrollCheckBox->setChecked(enabled);
}