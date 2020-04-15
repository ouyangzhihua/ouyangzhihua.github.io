---
layout: post
title: '视觉伺服控制的基础方法'
category: visual servoing 
---


视觉伺服控制是指通过利用计算机的视觉信息来控制机器人的动作，视觉信息通常是从机器人工作区的摄像机或者机器人本身上的摄像机获取的。视觉伺服控制将图像处理，计算视觉和控制理论结合起来形成的一种新的控制方法。


### 视觉伺服的基本组成
一般性的视觉伺服误差定义为：
$$\begin{equation}
e(x)=s(m(t),a)-s^{*} \label{1}
\end{equation}$$
其中$s$为视觉特征，$m(t)$为特征点的图像坐标，$a$为相机内部参数或3D模型。  

视觉伺服控制方法的区别都在于对$s$的设计，一旦$S$确定，控制方法就很容易得到了。最简单直接的方法就是直接设计一个速度控制器。记相机的空间速度为：$v=(v_{c},\omega_{c})$,则有：
$$\begin{equation}
\dot{s}=L_{s}v \label{2}
\end{equation}$$
其中$L_{s}$为交互矩阵或特征雅可比矩阵。由$\eqref{1}$和$\eqref(2)$可以得到相机速度和误差之间的关系:
$$\begin{equation}
\dot{e}=L_{e}v \label{3}
\end{equation}$$
其中$L_{s}=L_{e}$。


###控制部分
假设我们希望误差$e$能够以指数形式减小即：
$$\begin{equation}
\dot{e}=\lambda e \label{4}
\end{equation}$$
可得：
$$\begin{equation}
v_{c}=-\lambdaL_{e}^{+} \label{5}
\end{equation}$$
