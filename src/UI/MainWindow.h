#pragma once
#include "../Base/pidctl.h"
#include "../Workflow/Receiver.h"
#include "../Workflow/Workflow.h"
#include "ImageListWidget.h"
#include "LogWidget.h"
#include "MatchMatrixWidget.h"
#include "ModelViewerWidget.h"
#include "ProjectWidget.h"
#include "ReconstructionManagerWidget.h"
#include <QAction>
#include <QMainWindow>
#include <QMenu>
#include <QMutex>
#include <QToolBar>

class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	explicit MainWindow(QWidget* parent = nullptr);
	~MainWindow();

	void StartReceive();
	void StopReceive();
	void ProcessImage(const std::string& newImagePath);

protected:
	void closeEvent(QCloseEvent* event) override;

private slots:
	void NewProject();
	void ExportProject();
	void ChangeCurrentModel();
	void ShowMatchMatrix();
	void ChangeSelectModel();
	void RenderNow();
	void NewProject_SLOT();
	void onTabChanged(int index);
	void OnTrackModelToggled(bool checked);

	void addNewImageInWidget(QString p_imagePath);
	void UpdateStatus(const QString& status);

	void UpdateSimulationSpeed(quint32 read, quint32 reconstructed); // 更新影像模拟器传入速度

	// Feedback UI相关槽函数
	void OnShowMeshChanged();
	void OnShowQualityChanged();
	void OnShowGuidePathChanged();
	void OnFeedback();
	void OnRenderNow(quint32 p_lastRegistedImageID);

private:
	void SetupWidgets();
	void SetupActions();
	void SetupConnections() const;
	void CreateMenus();
	void CreateToolbar();
	void CreateStatusbar();
	void RunFunc();
	void ExportRunningProject();
	void ExportModelOnRegisted();
	void SetModelWidgetLastRegistedImageID(quint32 imageID);

	Receiver* receiver;
	Database* database;
	ReconstructionManager* reconstructionManager;
	WorkFlow* workflow;
	ModelViewerWidget* modelViewerWidget;
	ReconstructionManagerWidget* reconstructionManagerWidget;
	ImageListWidget* imageListWidget;
	LogWidget* logWidget;
	ProjectWidget* projectWidget;
	ShowMatchMatrixWidget* showMatchMatrixWidget;
	QTabWidget* tabWidget;
	QDockWidget* imageListWindowDock;
	QDockWidget* logWindowDock;
	QLabel* statusLabel;
	QAction* newProjectAction;
	QAction* exportProjectAction;
	QAction* startReceiveAction;
	QAction* stopReceiverAction;
	QAction* renderNowAction;
	QAction* showMatchMatrixAction;
	QAction* showMeshAction;
	QAction* showQualityAction;
	QAction* showGuidePathAction;
	QMenu* projectMenu;
	QMenu* transmitMenu;
	QMenu* feedbackMenu;
	QToolBar* projectToolBar;
	QToolBar* transmitToolBar;
	QToolBar* reconstructToolBar;
	QToolBar* feedbackToolBar;
	QCheckBox* isTrackModelCheckBox;

	std::atomic_int simulationSpeed = 800; // 初始发送影像的间隔时间，单位:ms
	BufferPIDController pidController;
};