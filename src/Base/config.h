#pragma once

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <memory>
#include <mutex>
#include <string>

// 配置项结构定义
struct SfMConfig {
	bool useFocalLength;
	double useFactor;
	std::string cameraModel;
};

struct PipelineConfig {
	bool AutoRunImages;
	bool AutoExportModel;
	int AutoExportFrame;
	int MinEstimatedImageNum;
};

struct PythonConfig {
	std::string PythonHome;
	std::string PythonWorkspace;
	std::string GlobalFeatureModel;
};

struct FeedbackConfig {
	int StartCreateMesh;
	int Step;
};

struct TestConfig {
	std::string ModelFolder;
	int Start;
	int Step;
	int End;
};

class Config {
public:
	static Config& getInstance(const std::string& iniPath = "config.ini") {
		static std::once_flag flag;
		static std::unique_ptr<Config> instance;
		std::call_once(flag, [&]() {
			instance.reset(new Config(iniPath));
			});
		return *instance;
	}

	const std::string& getIniPath() const {
		return m_iniPath;
	}

	SfMConfig sfm;
	PipelineConfig pipeline;
	PythonConfig python;
	FeedbackConfig feedback;
	TestConfig test;

	Config(const Config&) = delete;
	Config& operator=(const Config&) = delete;

private:
	std::string m_iniPath;

	Config(const std::string& iniPath) : m_iniPath(iniPath) {
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(iniPath, pt);

		// [SfM]
		sfm.useFocalLength = pt.get<bool>("SfM.useFocalLength", false);
		sfm.useFactor = pt.get<double>("SfM.useFactor", 0.704);
		sfm.cameraModel = pt.get<std::string>("SfM.cameraModel", "SIMPLE_RADIAL");

		// [Pipeline]
		pipeline.AutoRunImages = pt.get<bool>("Pipeline.AutoRunImages", true);
		pipeline.AutoExportModel = pt.get<bool>("Pipeline.AutoExportModel", false);
		pipeline.AutoExportFrame = pt.get<int>("Pipeline.AutoExportFrame", 1);
		pipeline.MinEstimatedImageNum = pt.get<int>("Pipeline.InitialImagesNum", 20);

		// [Python]
		python.PythonHome = pt.get<std::string>("Python.PythonHome", "C:\\Users\\WTG\\env\\Python");
		python.PythonWorkspace = pt.get<std::string>("Python.PythonWorkspace", "C:\\Users\\WTG\\code\\OntheFlySfM\\src\\Feature\\pyscripts");
		python.GlobalFeatureModel = pt.get<std::string>("Python.GlobalFeatureModel", "C:\\Users\\WTG\\code\\OntheFlySfM\\src\\Feature\\pyscripts");

		// [Feedback]
		feedback.StartCreateMesh = pt.get<int>("Feedback.StartCreateMesh", 4000);
		feedback.Step = pt.get<int>("Feedback.Step", 10);

		// [Test]
		test.ModelFolder = pt.get<std::string>("Test.ModelFolder", "D:\\DATASET\\whu_models");
		test.Start = pt.get<int>("Test.Start", 80);
		test.Step = pt.get<int>("Test.Step", 10);
		test.End = pt.get<int>("Test.End", 110);
	}
};