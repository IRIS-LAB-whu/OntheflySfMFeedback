# On-the-fly Feedback SfM

**On-the-fly Feedback SfM:** An Explore-and-Exploit Framework for Real-Time UAV Photogrammetry.

### Key Features

* **Online coarse mesh generation** from incrementally expanding sparse point clouds.
* **Real-time mesh quality assessment** with key quality metrics (GSD, observation redundancy, and reprojection error).
* **Predictive path planning & trajectory refinement** for optimized navigation guidance.


---

# Install Instructions

## Environment Setup

* Visual Studio 2022 (v17.9.1 recommended)
* Qt 5.12.12 (Core, Gui, OpenGL, Widgets)
* CUDA 12.2
* VCPKG

  * Run `vcpkg integrate install`
  * Install C++ dependencies via `vcpkg install`
* Intel® oneAPI Threading Building Blocks (TBB)
* Python environment

  * Install PyTorch:

    ```bash
    pip install torch==2.2.2 torchvision==0.17.2 torchaudio==2.2.2 --index-url https://download.pytorch.org/whl/cu118
    ```
  * Install dependencies:

    ```bash
    pip install -r requirements.txt
    ```
  * Install `deep-image-matching`

## Build Instructions

### Build Order

Please strictly follow the order below to ensure correct dependency linking:

#### Third-party Libraries (`thirdparty`)

* SIFTGPU
* VLFeat

#### Internal Modules

* Base
* Geometry
* Scene
* Estimator
* Feature
* Workflow
* UI

