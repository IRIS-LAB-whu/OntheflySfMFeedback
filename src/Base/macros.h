#pragma once

////////////////////////////////////////////////////////////////////////////////
// On-the-Fly 系统配置宏
// 使用以下宏定义控制编译时行为及流程选择
////////////////////////////////////////////////////////////////////////////////

// 重建过程控制
#define NO_MERGE_ON_RECONSTRUCTION		// 禁止在三维重建过程中进行模型合并操作
#define IMAGE_LIST_NOT_FOLLOW			// 图像列表不跟随主窗口刷新
// #define NO_MERGE						// 全局禁用模型合并功能
// #define AUTO_RUN_IMAGES				// 自动运行图像处理流程，无需手动干预
#define GaussianSplatting_Output		// 逐张注册影像输出重建结果

// 算法特性选择
// #define NOT_USE_GLOBAL_FEATURES		// 不使用全局特征进行匹配或重建
#define USE_HIERARCHICAL_WEIGHT_BA		// 使用分层加权BA（Bundle Adjustment）
// #define Superpoint_Feature			// 使用SuperPoint特征提取算法
// #define Lightglue_Match				// 使用LightGlue特征匹配算法
// #define Similar						// 启用相似纹理剔除相关功能

// 系统功能配置
// #define DISABLE_MODELVIEWER			// 禁用模型查看器
// #define DEMO_MODE					// 启用演示模式，简化功能或添加演示逻辑

////////////////////////////////////////////////////////////////////////////////
// 注：取消注释相应宏以启用相应功能，注释或删除以禁用
////////////////////////////////////////////////////////////////////////////////