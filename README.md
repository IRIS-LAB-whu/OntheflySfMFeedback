# On_the_fly_SfM
On-the-fly SfM: running online SfM while image capturing!
Similar to conventional SfM, On-the-fly SfM yields image poses and 3D sparse points, but we do this while the image capture. More specifically, the current image’s pose and corresponding 3D points can be estimated before next image is captured and on-the-fly to be processed, i.e., what you capture is what you get.



# Install Instructions
## 环境准备
- 安装 Visual Studio 2022 （V17.9.1）
- 安装 Qt 5.12.12，核心组件Core, Gui, OpenGL, Widgets
- 安装 CUDA 12.2
- 安装 VCPKG  
  - 运行 `vcpkg integrate install` 集成到MSBuild中  
  - 安装c++环境 `vcpkg install`
- 安装 TBB库：Intel® oneAPI Threading Building Blocks
- 安装 Python环境  
  - 安装Pytorch `pip install torch==2.2.2 torchvision==0.17.2 torchaudio==2.2.2 --index-url https://download.pytorch.org/whl/cu118`  
  - 安装依赖项 `pip install -r requirements.txt`  
  - 安装deep-image-matching

## 构建项目
### 构建顺序
代码经过项目依赖优化，请严格按照下面的顺序进行构建
#### 第三方库thirdparty
- SIFTGPU
- VLFeat
#### 内部库
- Base
- Geometry
- Scene
- Estimator
- Feature
- Workflow
- UI
