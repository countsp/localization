**传统感知方案**

传统的感知算法就是输入一张图片，输出检测分割的结果，例如OpenCV。

但对于自动驾驶这种复杂任务来说，单张图片的 2D 检测是无法满足要求的。

一方面单个相机视角有限，另一方面单张图很难获得准确的 3D 信息。

![传统感知方案](https://mmbiz.qpic.cn/sz_mmbiz_jpg/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZXH7Ia9r3JupG18w0cQx1ibM0yBmQ1WfEGbrVF2pANu2bbmpQZCNLrhQ/640?wx_fmt=jpeg&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

它首先在多个相机或者雷达上独立执行感知任务，随后将这些独立感知结果融合在一起。这是一项极具挑战的任务。由于每个相机具有独特的视野和观察结果，这使得整个系统的复杂性大大增加。在实际应用中，由于各种因素的影响，单个相机的检测结果可能并不准确，例如物体可能被截断，或者由于环境因素如光线、阴影等的影响，使得识别结果出现误差。此外，不同模态数据之间的融合，需要人工设计复杂的规则，给系统带来较高的复杂度。

因此需要多传感器融合。

**BEV**

BEV采用的是一种鸟瞰视图的融合方式。

![BEV](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZBsfXE4tyFoqzBj4gCoKquUQcYuUIhSsT5qeBfictwWjBdHP6q7wCN4Q/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

**BEV融合策略**

无论是哪种 BEV 算法，本质都是把不同传感器的输出转换到统一的 BEV 空间，即鸟瞰视图中，之后在 BEV 空间内获得完整的感知结果。

BEV 有三种典型的融合策略。

1. 传感器后融合（目标级融合）

所谓后融合，是指各传感器针对目标物体单独进行深度学习模型推理，从而各自输出带有传感器自身属性的结果，并在决策层进行融合。

![后融合](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZwkAKeN8zbOBZvq6ibnIgt6BicIngLADRoy3RiaLYVYuIpBmhBpk1lItcQ/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)


其优势是不同的传感器都独立进行目标识别，解耦性好，且各传感器可以互为冗余备份。

后融合和传统的感知算法流程基本一致，因此也面临传统算法的问题：各自传感器经过目标识别再进行融合时，中间损失了很多有效信息，影响了感知精度，而且最终的融合算法，仍然是一种基于规则的方法，要根据先验知识来设定传感器的置信度，局限性很明显。

2. 传感器前融合（数据级融合）

![前融合](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZGUibSAE0YNWsic95zUkxBsgOHzNJrlMfzwW2G5ClodDlibyCiaibFl6A1MA/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

其优势是可以从整体上来处理信息，让数据更早做融合，从而让数据更有关联性，比如把激光雷达的点云数据和摄像头的像素级数据进行融合，数据的损失也比较少。

不过其挑战也很明显，因为视觉数据和激光雷达点云数据是异构数据，其坐标系不同，视觉数据是 2D 图像空间，而激光雷达点云是 3D 空间，在进行融合时，只能在图像空间里把点云放进去，给图像提供深度信息，或者在点云坐标系里，通过给点云染色或做特征渲染，而让点云具有更丰富的语义信息。这需要复杂的融合策略，对算力要求也很高。

3. 传感器中融合（特征级融合）

所谓中融合，就是先将各个传感器通过神经网络模型提取中间层特征（即有效特征），再对多种传感器的有效主要特征进行融合，从而更有可能得到最佳推理。

对有效特征在 BEV 空间进行融合，一来数据损失少，二来算力消耗也较少（相对于前融合），所以一般在 BEV 空间进行中融合比较多。

![BEV](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZlZibXbRScQrtu3z9jE2QLAWicHqoGyBndrib8dV8JKrWXQzItkVbIgudA/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)


融合方法（2D-3D映射）

传统方法通常采用逆透视变换（IPM）求得相机平面到地平面的单应性矩阵，实现平面到平面的转换，再进行多视角图像的拼接。

![IPM](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZeLg0lqZ6lgqctYT2HDeE3zYzZQ5G1PLXyibjuAtYI9yjTVMQSLVPA0Q/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)

局限性：传统 IPM 算法依赖相机参数标定的准确性，并且从原理上说，IPM 只能表征地平面的信息，有一定高度的目标都会在图片上产生畸变，所以同样需要假设地面平坦、目标接地，这就意味着难以应用在较远距离的感知任务中。

近年来，很多研究也开始采用深度模型（主要是 transformer）让网络自己学习特征融合（可以理解为隐式映射，主要应用在中融合方式）。

BEV把整个感知框架都统一起来了。我们只要把不同传感器的输入都丢给 transformer，它就能输出一个 BEV 空间下的 feature，这个 feature 可以同时用于后续的物体检测、车道线分割、实时建图、轨迹预测等等任务。而这些感知输出由于在同一个空间中，又可以进一步被后续的规控任务使用。

![BEV](https://mmbiz.qpic.cn/sz_mmbiz_png/bdpnCavfx2pO7wPkxmXT7QKHJybATVBZOkaMfWnnAIxMNsDMd5yjKo1NmwHvPKz8StkQesW0EZLibwuu4NGlJGA/640?wx_fmt=png&from=appmsg&tp=webp&wxfrom=5&wx_lazy=1&wx_co=1)
