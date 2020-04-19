---
layout: post
title: '视觉伺服控制的基础方法'
modal-id: 1
date: 2020-04-13
img: safe.png
project-date: April 2020
client: OYZH
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

其中$L_{s}$为交互矩阵或特征雅可比矩阵。由上式可以得到相机速度和误差之间的关系:

$$\begin{equation}
	\dot{e}=L_{e}v \label{3}
\end{equation}$$

其中$L_{s}=L_{e}$。


### 控制部分
假设我们希望误差$e$能够以指数形式减小即：

$$\begin{equation}
	\dot{e}=\lambda e \label{4}
\end{equation}$$

可得：

$$\begin{equation}
	v_{c}=-\lambda L_{e}^{+} \label{5}
\end{equation}$$

由上式可以得到相机速度和误差之间的关系:

$$\begin{equation}
\dot{e}=L_{e}v 
\end{equation}$$

其中$L_{s}=L_{e}$。

#### 控制律
假设我们希望误差$e$能够以指数形式减小即：

$$\begin{equation}
\dot{e}=\lambda e 
\end{equation}$$

可得：

$$\begin{equation}
   v_{c}=-\lambda L_{e}^{+} 
\end{equation}$$

其中$L_{e}^{+}$是$L_{e}$的广义逆矩阵或伪逆。$L_{e}^{+}$或$L_{e}$一般无法直接得到因此须设计逼近器估计$L_{e}^{+}$，记为$\hat{L_{e}^{+}}$。
实际的控制律即为：

$$\begin{equation}
	v_{c}=-\lambda \hat{L_{e}^{+}}
\end{equation}$$

#### 交互矩阵估计器
有三种常见的估计器形式：

$$\begin{align}
	\hat{L_{e}^{+}} &= L_{e^{*}}^{+}	\\
	\hat{L_{e}^{+}} &= L_{e}^{+}	\\
	\hat{L_{e}^{+}} &= 1/2(L_{e^{*}} + L_{e})^{+}
\end{align}$$

$L_{e^{*}}^{+}$是期望点的交互矩阵，$L_{e}^{+}$是当前点的交互矩阵。

### matlab仿真
#### IBVS

camera模块：将世界点坐标投影到图像平面

pose integer模块：更新相机的位姿

运行代码：
```matlab
%IBVS
clear
cam = CentralCamera('default'); %定义一个默认相机
P = mkgrid( 2, 0.5, 'T', transl(0,0,3) ); %形成边长为0.5的正方形，中心在（0，0，3）  
s_star = bsxfun(@plus, 100*[-1 -1 1 1; -1 1 1 -1], cam.pp'); %目标特征，中心在主点，400*400像素正方形
cam_ip = transl(1,1,-3) * trotz(0.6);     %相机初始位姿
depth = 3;      %深度
sim('viserv_ibvs');    %运行仿真
camtraj = transl(T);    %将齐次矩阵转换为坐标
camrow = tr2rpy(T);     %将齐次矩阵转化为对应的横滚俯仰偏航角

%画图
figure(2);
plot3(camtraj(:,1),camtraj(:,2),camtraj(:,3));   %绘制相机运动轨迹
xlabel('X轴');ylabel('Y轴');zlabel('Z轴');
title('相机轨迹');
text(1,1,-3,'始点');
grid on         %打开坐标网格
figure(3);
plot(t,camrow(:,1),'-b','linewidth',1);       %绘制相机旋转角度变化
hold on
plot(t,camrow(:,2),'*g','linewidth',1);
hold on
plot(t,camrow(:,3),'-.r','linewidth',1);
xlabel('time(s)');
ylabel('横滚俯仰偏航角');
legend('R', 'P', 'Y');      %添加图例
figure(4);
plot(t,v_c,'linewidth',1);       %绘制相机速度
xlabel('time(s)');
ylabel('相机速度');
legend('v_{x}', 'v_{y}', 'v_{z}','\omega_{x}', '\omega_{y}', '\omega_{z}');      %添加图例,使用tex格式显示公式
```
分别对前面三种不同的交互矩阵估计器进行仿真：
<img src="/img/post_img/ibvs_model.png" alt="模型" width="50%" height="50%" />

pose estimation模块：对相机的位姿进行估计

control law模块：速度控制器,表达式为：

$$\begin{align}
v_{c} &= -\lambda ((^{c^{*}}t_{o} - ^{c}t_{o}) + [^{c}t_{o}]_{\times} \theta u) \label{vc} \\
\omega_{c} &= -\lambda \theta u  \label{wc}
\end{align}$$

运行代码：
```matlab
% PBVS控制
clear
cam = CentralCamera('default');%定义一个参数已知的相机
P = mkgrid( 2, 0.5, 'T', transl(0,0,3) ); %目标有四点正方形
T_star = transl(0, 0, 0.5);%期望的相对相机的目标位姿，目标位于相机前方1米处
cam_ip = transl(1,1,-3)*trotz(0.6);%相机初始位姿
lambda = -1;

viserv_pbvs
fig = get_param('viserv_pbvs','Handle');
saveas(fig,'pbvs_model.png');   %保存模型图
sim('viserv_pbvs');
camtraj = transl(T);    %将齐次矩阵转换为坐标
camrow = tr2rpy(T);     %将齐次矩阵转化为对应的横滚俯仰偏航角

%画图
figure(2);
plot3(camtraj(:,1),camtraj(:,2),camtraj(:,3));   %绘制相机运动轨迹
xlabel('X轴');ylabel('Y轴');zlabel('Z轴');
title('相机轨迹');
text(1,1,-3,'始点');
grid on         %打开坐标网格
saveas(gcf,'traj1_pbvs','epsc');
figure(3);
plot(t,camrow(:,1),'-b','linewidth',1);       %绘制相机旋转角度变化
hold on
plot(t,camrow(:,2),'*g','linewidth',1);
hold on
plot(t,camrow(:,3),'-.r','linewidth',1);
xlabel('time(s)');
ylabel('横滚俯仰偏航角');
legend('R', 'P', 'Y');      %添加图例
saveas(gcf,'row1_pbvs','epsc');
figure(4);
plot(t,v_c,'linewidth',1);       %绘制相机速度
xlabel('time(s)');
ylabel('相机速度');
legend('v_{x}', 'v_{y}', 'v_{z}','\omega_{x}', '\omega_{y}', '\omega_{z}');      %添加图例,使用tex格式显示公式
saveas(gcf,'v1_pbvs','epsc');
```



