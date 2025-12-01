#pragma once
#include <iostream>
#include <thread>
#include <mutex>
#undef slots
#include <cmath>
#include <Python.h>
#include "../Lib/site-packages/numpy/core/include/numpy/arrayobject.h"
#define slots Q_SLOTS


class PyLoader final {
public:
	explicit PyLoader();
	~PyLoader();
	void Initialize();
	static PyObject* GlobalExtractInstance;
	static PyObject* DeepImageMatchingInstance;

	static PyObject* GlobalExtractorFunc;
	static PyObject* LocalExtractorFunc;
	static PyObject* LocalFeatureSaveFunc;
	static PyObject* LocalFeatureReadFunc;
	static PyObject* LocalMatcherFunc;

	static PyThreadState* initThreadState;
	static std::mutex mtx;
};

class PythonThreadLocker
{
	PyGILState_STATE state;
public:
	PythonThreadLocker() {
		PyLoader::mtx.lock();
		state = PyGILState_Ensure();
	}
	~PythonThreadLocker() {
		PyGILState_Release(state);
		PyLoader::mtx.unlock();
	}
};