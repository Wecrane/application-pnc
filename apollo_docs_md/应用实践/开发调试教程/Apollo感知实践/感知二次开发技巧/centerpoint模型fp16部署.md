---
title: centerpoint模型fp16部署
source: https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xBA_x94_xE7_x94_xA8_xE5_xAE_x9E_xE8_xB7_xB5_2_xE5_xBC_x80_xE5_x8F_x91_xE8_xB0_x83_25863effa3253dd94590c818de1cdf9c.html
category: 应用实践 > 开发调试教程 > Apollo感知实践 > 感知二次开发技巧 > centerpoint模型fp16部署
---

# centerpoint模型fp16部署

### paddle inference推理接口flag解释

// lidar_center_point
DECLARE_bool(use_trt);  // 是否使用tensorrt推理接口
DECLARE_int32(trt_precision); // fp32、int8以及fp16推理精度标志位 , 0: kFloat32, 1: kInt8, 2: kHalf
DECLARE_int32(trt_use_static); // 是否使用序列化文件，1代表使用，首次加载会生成序列化文件，后续再次加载会反序列化该文件，提升模型加载速度
DECLARE_bool(use_calibration); // 是否使用标定表，int8精度推理时需要开启
DECLARE_bool(use_dynamicshape); // 是否加载动态图文件
DECLARE_bool(collect_shape_info); // 收集动态图文件
DECLARE_string(dynamic_shape_file); // 动态图文件生成以及读取路径
fragment

### 如何使用（生成动态图文件以及序列化文件）：

#### 为什么需要使用生成的动态图：

PaddlePaddle采用子图的形式对TensorRT进行集成，当模型加载后，神经网络可以表示为由变量和运算节点组成的计算图。Paddle TensorRT实现的功能是对整个图进行扫描，发现图中可以使用TensorRT优化的子图，并使用TensorRT节点替换它们。在模型的推断期间，如果遇到TensorRT节点，Paddle会调用TensorRT库对该节点进行优化，其他的节点调用Paddle的原生实现。TensorRT在推断期间能够进行Op的横向和纵向融合，过滤掉冗余的Op，并对特定平台下的特定的Op选择合适的kernel等进行优化，能够加快模型的预测速度。

#### 步骤一：生成动态图

将flag设置如下：

###########################################################################
# centerpoint paddle inference flag
--use_trt=true
--trt_precision=2
--collect_shape_info=true
--use_dynamicshape=false
fragment

注意：collect_shape_info和use_dynamicshape不能同时设置为true开启，应该先收集动态图再使用生成出的动态图文件。

flag设置完成后，启动lidar感知流程，并播放数据包，程序结束后动态图文件会存放在开发者自己设置的dynamic_shape_file路径下。

cyber_launch start modules/perception/launch/perception_lidar.launch
fragment

tips:播放数据包收集到的动态图文件范围可能会有限，这样加载该动态图文件的模型实际运行时可能会出现报错，建议在实车测试环境下运行上述lidar感知流程，收集更加具体的动态图范围。

#### 步骤二：加载生成的动态图文件

flag设置如下：建议将trt_use_static设置为1，如果不使用生成好的序列化文件，每次模型加载时间很长（约5分钟左右），trt_use_static设置为1，第一次模型加载时间较长，但是会生成相应的序列化文件，生成的序列化文件目录为_opt_cache，该目录自动放在模型目录路径下，后续模型加载时会直接反序列化该目录中的文件，极大程度降低模型加载时间（约5秒钟）。

###########################################################################
# centerpoint paddle inference flag
--use_trt=true
--trt_precision=2
--trt_use_static=1
--collect_shape_info=false
--use_dynamicshape=true
fragment

注意：如果想要使用int8量化方式，将步骤一中的trt_precision值改为1即可。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。