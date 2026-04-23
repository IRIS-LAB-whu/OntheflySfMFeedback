<h1 align="center">On-the-fly Feedback SfM: Online Explore-and-Exploit UAV Photogrammetry with Incremental Mesh Quality-Aware Indicator and Predictive Path Planning</h1>

<p align="center">
  Liyuan Lou<sup>†</sup>, Wanyun Li<sup>†</sup>, Wentian Gan<sup>†</sup>, Yifei Yu, Tengfei Wang, Xin Wang, <i>Member, IEEE</i>, Zongqian Zhan, <i>Member, IEEE</i>
</p>

<p align="center">
  <i>School of Geodesy and Geomatics, Wuhan University, Wuhan 430079, China</i><br/>
  <sup>†</sup> Equal contribution &nbsp;|&nbsp;
  Corresponding author: <a href="mailto:xwang@sgg.whu.edu.cn">Xin Wang</a>
</p>

<p align="center">
  <a href="https://github.com/IRIS-LAB-whu">
    <img src="https://img.shields.io/badge/Lab-IRIS--LAB--whu-blue?logo=github" alt="IRIS Lab"/>
  </a>
  &nbsp;
  <a href="https://arxiv.org/abs/2512.02375">
    <img src="https://img.shields.io/badge/arXiv-2512.02375-b31b1b.svg" alt="arXiv"/>
  </a>
  &nbsp;
  <a href="https://louliyuan.github.io/OntheflyFeedbackSfM_homepage/">
    <img src="https://img.shields.io/badge/Project-Page-brightgreen" alt="Project Page"/>
  </a>
</p>

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

## Datasets

We evaluate our method on the following UAV datasets:

| Dataset | Images | Platform | Resolution | Source |
|---------|--------|----------|------------|--------|
| SHHY | 770 | DJI Mavic 2 Pro | 1920×1080 | Self-captured |
| PHANTOM | 467 | DJI Mavic 2 Pro | 1920×1080 | Bu et al., 2016 |
| US3D | 990 | — | 5472×3648 | Lin et al., 2022 (UrbanScene3D) |
| GYM | 580 | DJI Matrice 4T | 4032×3024 | Self-captured |
| YS | 320 | DJI Matrice 4T | 4032×3024 | Self-captured |
| XingHu | — | DJI Matrice 4T | 4032×3024 | Self-captured |

---

# Install Instructions

## Prerequisites

> **Platform: Windows only** (requires Visual Studio 2022)

> **Recommended hardware:** Intel i9-12900K CPU, NVIDIA RTX3080; **Minimum GPU:** NVIDIA GTX 1080 Ti, CUDA 12.2

This project is built upon **[SfM on-the-fly](https://www.sciencedirect.com/science/article/abs/pii/S0924271625001388)** (Zhan et al., ISPRS Journal of Photogrammetry and Remote Sensing, 2025). Please ensure you are familiar with its setup before proceeding.

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

---

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
```

---

## Acknowledgments                                                                                                                                         
                                                                        
This work was supported by the National Natural Science Foundation of China (No. 42301507) and the Luojia Undergraduate Innovation Research Fund of Wuhan  
University.                                                             
                                                                                                                                                           
**Corresponding authors:** Xin Wang (xwang@sgg.whu.edu.cn) and Zongqian Zhan (zqzhan@sgg.whu.edu.cn)
