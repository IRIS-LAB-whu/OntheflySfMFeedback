#include "../Base/config.h"
#include "filesystem"
#include "glog/logging.h"
#include "PyLoader.h"
#include <codecvt>
#include <locale>

using namespace std;
namespace fs = std::filesystem;

PyObject* PyLoader::DeepImageMatchingInstance = nullptr;
PyObject* PyLoader::GlobalExtractInstance = nullptr;

PyObject* PyLoader::GlobalExtractorFunc = nullptr;
PyObject* PyLoader::LocalExtractorFunc = nullptr;
PyObject* PyLoader::LocalFeatureReadFunc = nullptr;
PyObject* PyLoader::LocalFeatureSaveFunc = nullptr;
PyObject* PyLoader::LocalMatcherFunc = nullptr;

PyThreadState* PyLoader::initThreadState;
std::mutex PyLoader::mtx;

PyLoader::PyLoader()
{
    //初始化Python，设置编译器路径以及环境变量
    LOG(INFO)<< "Initializing PyTorch environment...";
    auto& config = Config::getInstance();
    string pythonHome = config.python.PythonHome;
    string pythonWorkspace = config.python.PythonWorkspace;
    fs::path pythonHomePath = fs::absolute(pythonHome);
    fs::path pythonWorkspacePath = fs::absolute(pythonWorkspace);
    fs::path libSitePackages = pythonHomePath / "Lib" / "site-packages";
    fs::path dlls = pythonHomePath / "DLLs";
    fs::path lib = pythonHomePath / "Lib";
    string pythonHomeAbsolute = pythonHomePath.string();
    string pythonPaths =
        libSitePackages.string() + ";" +
        dlls.string() + ";" +
        lib.string() + ";" +
        pythonWorkspacePath.string();
    wstring wPythonHome(pythonHomeAbsolute.begin() , pythonHomeAbsolute.end());
    wstring wPythonPaths(pythonPaths.begin() , pythonPaths.end());

    PyStatus status;
    PyConfig pyconfig;
    PyConfig_InitPythonConfig(&pyconfig);
    pyconfig.isolated = 1;
    PyConfig_SetString(&pyconfig, &pyconfig.home , wPythonHome.c_str());
    PyConfig_SetString(&pyconfig, &pyconfig.pythonpath_env , wPythonPaths.c_str());
    status = Py_InitializeFromConfig(&pyconfig);
    PyConfig_Clear(&pyconfig);

    const char* version = Py_GetVersion();
    LOG(INFO) << "Python version: " + string(version);

    Initialize();

    initThreadState = PyEval_SaveThread();
}

void PyLoader::Initialize()
{
    _import_array();

    PyObject* GlobalExtractorModule = PyImport_ImportModule("GlobalFeatureExtractor");
    if (GlobalExtractorModule == NULL) {
        PyErr_Print();
        LOG(INFO) << "Module GlobalFeatureExtractor not found.";
        return;
    }

    PyObject* globalFeatureExtractorClass = PyObject_GetAttrString(GlobalExtractorModule , "GlobalFeatureExtractor");
    if (!globalFeatureExtractorClass)
    {
        LOG(WARNING)<<"Failed to get GlobalFeatureExtractor class!";
        PyErr_Print();
        return;
    }

    PyObject* args = PyTuple_New(4);
    auto& config = Config::getInstance();
    string modelHome = config.python.GlobalFeatureModel;
    fs::path modelHomePath = fs::absolute(modelHome);
    fs::path vggpth = modelHomePath / "vgg16-397923af.pth";
    fs::path vgg16_64pth = modelHomePath / "VGG16_64_desc_cen.hdf5";
    fs::path vgg_netvladpth = modelHomePath / "VGG16_NetVlad_NoSplit.pth.tar";
    fs::path pca_dimspth = modelHomePath / "PCA_dims32768to2048.model";
    PyTuple_SetItem(args , 0 , Py_BuildValue("s" , vggpth.string().c_str()));
    PyTuple_SetItem(args , 1 , Py_BuildValue("s" , vgg16_64pth.string().c_str()));
    PyTuple_SetItem(args , 2 , Py_BuildValue("s" , vgg_netvladpth.string().c_str()));
    PyTuple_SetItem(args , 3 , Py_BuildValue("s" , pca_dimspth.string().c_str()));
    PyObject* globalFeatureExtractorClassInstantiation = PyObject_CallObject(globalFeatureExtractorClass , args);
    Py_DECREF(args); Py_DECREF(globalFeatureExtractorClass);
    if (!globalFeatureExtractorClassInstantiation)
    {
        LOG(WARNING) << "Instantiation failed!";
        PyErr_Print();
        return;
    }
    GlobalExtractInstance = globalFeatureExtractorClassInstantiation;
    GlobalExtractorFunc = PyObject_GetAttrString(globalFeatureExtractorClassInstantiation , "Extract");
    if (!GlobalExtractorFunc)
    {
        LOG(WARNING)<<"Get function failed!";
        PyErr_Print();
        return;
    }
    LOG(INFO) << "The global extractor has been successfully loaded!";


    /*PyObject* DIMExtractModule = PyImport_ImportModule("DIMScript");
    if (DIMExtractModule == NULL) {
        PyErr_Print();
        LOG(WARNING) << "Module DIMScript not found.";
        return;
    }
    PyObject* DIMExtractClass = PyObject_GetAttrString(DIMExtractModule , "OTFMatcher");
    if (DIMExtractClass == NULL) {
        PyErr_Print();
        LOG(WARNING) << "Class OTFMatcher not found.";
        return;
    }
    PyObject* DIMExtractClassInstantiation = PyObject_CallObject(DIMExtractClass , nullptr);
    Py_XDECREF(DIMExtractClass);
    if (DIMExtractClassInstantiation == NULL) {
        PyErr_Print();
        LOG(WARNING) << "Instance OTFMatcher create failed.";
        return;
    }
    DeepImageMatchingInstance = DIMExtractClassInstantiation;
    LocalExtractorFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "extract");
    if (!LocalExtractorFunc)
    {
        LOG(WARNING) << "Get function failed!";
        PyErr_Print();
        return;
    }
    LocalFeatureSaveFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "save_features");
    if (!LocalFeatureSaveFunc)
    {
        LOG(WARNING) << "Get function failed!";
        PyErr_Print();
        return;
    }
    LOG(INFO) << "The local extractor has been successfully loaded!";

    LocalMatcherFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "match");
    if (!LocalMatcherFunc)
    {
        LOG(WARNING) << "Get function failed!";
        PyErr_Print();
        return;
    }
    LocalFeatureReadFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "read_features");
    if (!LocalFeatureReadFunc)
    {
        LOG(WARNING) << "Get function failed!";
        PyErr_Print();
        return;
    }*/
}

PyLoader::~PyLoader() {
    PyEval_RestoreThread(initThreadState);
    Py_Finalize();
}