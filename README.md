<h2 align="center">On-the-fly Feedback SfM: Online Explore-and-Exploit UAV Photogrammetry with Incremental Mesh Quality-Aware Indicator and Predictive Path Planning</h2>

<p align="center">
  Liyuan Lou<sup>†</sup>, Wanyun Li<sup>†</sup>, Wentian Gan<sup>†</sup>, Yifei Yu, Tengfei Wang, Xin Wang, <i>Member, IEEE</i>, Zongqian Zhan, <i>Member, IEEE</i>
</p>

[![arXiv](https://img.shields.io/badge/arXiv-2512.02375-b31b1b.svg)](https://arxiv.org/abs/2512.02375)

**On-the-fly Feedback SfM** is a real-time UAV photogrammetry framework that integrates **incremental Structure-from-Motion (SfM)**, **online coarse mesh generation**, **real-time mesh quality assessment**, and **predictive path planning** into a closed feedback loop for adaptive aerial data acquisition.

Unlike conventional offline photogrammetry pipelines that reconstruct scenes only after the flight is finished, this framework continuously evaluates the evolving reconstruction quality during image acquisition and actively guides the UAV toward under-observed or low-quality regions. This enables an **explore-and-exploit** workflow for efficient, reconstruction-aware, and navigation-guided UAV photogrammetry.
<img width="7816" height="4670" alt="workflow" src="https://github.com/user-attachments/assets/f441ee69-4dac-4fb4-810f-627d121a74d5" />

---

## Highlights

- **Incremental SfM with online feedback**  
  Continuously estimates camera poses and expands sparse 3D structure as new UAV images arrive.

- **Online coarse mesh generation**  
  Builds coarse mesh representations directly from incrementally reconstructed sparse point clouds.

- **Real-time mesh quality assessment**  
  Evaluates reconstruction quality using multiple indicators, including:
  - Ground Sampling Distance (**GSD**)
  - Observation Redundancy
  - Reprojection Error

- **Predictive path planning and trajectory refinement**  
  Detects low-quality mesh regions and generates informative next-best viewpoints for adaptive image acquisition.

- **Closed-loop UAV photogrammetry**  
  Couples scene reconstruction, quality evaluation, and navigation guidance into a unified online framework.

---
## Framework Overview

The system follows an **explore-and-exploit** strategy:

1. **Explore**  
   Acquire UAV images incrementally and perform online SfM to estimate camera poses and sparse 3D structure.

2. **Evaluate**  
   Generate a coarse mesh from the evolving sparse point cloud and assess its reconstruction quality in real time.

3. **Exploit**  
   Identify low-quality or under-observed regions and generate candidate viewpoints to improve reconstruction completeness and reliability.

4. **Refine**  
   Optimize the UAV trajectory to balance acquisition efficiency, path smoothness, and reconstruction-oriented coverage.

This design allows the UAV to adapt its flight path according to the current reconstruction status rather than relying solely on a predefined flight route.


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

## Citation

If you find this work useful in your research, please consider citing:

```bibtex
@misc{lou2025ontheflyfeedbacksfm,
  title={On-the-fly Feedback SfM: Online Explore-and-Exploit UAV Photogrammetry with Incremental Mesh Quality-Aware Indicator and Predictive Path Planning},
  author={Liyuan Lou and Wanyun Li and Wentian Gan and Yifei Yu and Tengfei Wang and Xin Wang and Zongqian Zhan},
  year={2025},
  eprint={2512.02375},
  archivePrefix={arXiv},
  primaryClass={cs.CV},
  doi={10.48550/arXiv.2512.02375},
  url={https://arxiv.org/abs/2512.02375}
}


