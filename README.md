# GAMES101_HW_TEAFOX

* ***Homeworks-for-GAMES101-计算机图形学入门-闫令琪***
* 根目录下文件夹对应前七次作业，每个文件夹中存放着Visual Studio清理后的解决方案,   
作业源码以及作业结果，PA0\*文件中即是作业源码和用到的其他Resource.  
***
## 环境
* **语言**: C++ 17
* **外部依赖库**: 部分作业用到了Eigen库和OpenCV
* **编译器**: VisualStudio 2017
***
## 目录
1. **SimpleTriangle**  
熟悉MVP矩阵变换，渲染一个简单的三角形

2. **TriangleWithDepth**  
熟悉DepthBuffer和光栅化中的插值，理解反走样技术，渲染一前一后两个三角形

3. **ShadingAndTexture**  
熟悉光栅化中的着色，以及着色中的各种概念，  
实现Normal，Texture  Sampling，Blinnphong, Bump mapping 和Displacement mapping着色

4. **BezierCurve**  
用de Casteljau实现n个控制点的贝塞尔曲线绘制

5. **Whitted_Raytracing**  
实现Whitted Style光线追踪 

6. **Whitted_Raytracing_BVH**    
实现用BVH加速Whitted_Style光线追踪  

7. **Path_Tracing**  
蒙特卡洛方法实现Path_Tracing with BVH