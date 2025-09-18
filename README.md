# eg: 3D->2D (u,v)

已知  
- 世界系点 $P_w=(2,3,10)$  
- 相机内参  

$$
K=\begin{bmatrix}
800 & 0 & 320 \\
0 & 800 & 240 \\
0 & 0 & 1
\end{bmatrix}
$$  

- 无旋转 $R=I_3$  
- 相机中心在世界系 $C=(0,0,-1)$  

外参换算：

$$
X_c = R X_w + t, \quad t=-RC=-C=(0,0,1)
$$  

因此相机系坐标：

$$
X_c = (2,3,10) + (0,0,1) = (2,3,11)
$$  

---

## 方法一：直接公式 $K(RP_w+t)$

计算：

$$
\begin{aligned}
\tilde{p} &= K (R P_w + t) \\
&= 
\begin{bmatrix}
800 & 0 & 320 \\
0 & 800 & 240 \\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
2 \\
3 \\
11
\end{bmatrix} \\
&=
\begin{bmatrix}
5120 \\
5040 \\
11
\end{bmatrix}
\end{aligned}
$$  

归一化得到：

$$
u = \frac{5120}{11} \approx 465.45, \quad
v = \frac{5040}{11} \approx 458.18
$$  

---

## 方法二：齐次矩阵公式 $K[R|t]P_w$

$$
[R|t] =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & 1
\end{bmatrix}, \quad
\tilde{P}_w=\begin{bmatrix}2 \\
3\\
10\\
1\\end{bmatrix}
$$  

计算：

$$
[R|t]\tilde{P}_w = \begin{bmatrix}2\\
3\\
11\end{bmatrix}
$$  

$$
\tilde{p} =
\begin{bmatrix}
800 & 0 & 320 \\
0 & 800 & 240 \\
0 & 0 & 1 
\end{bmatrix}
\begin{bmatrix}2\\
3\\
11\end{bmatrix}
\=
\begin{bmatrix}
5120 \\
5040 \\
11
\end{bmatrix}
$$  

归一化：  

$$
u = \frac{5120}{11} \approx 465.45, \quad
v = \frac{5040}{11} \approx 458.18
$$  

---

## ✅ 最终结果

两种方法得到一致的投影坐标：  

$$
(u,v) = \left(\tfrac{5120}{11}, \tfrac{5040}{11}\right) \approx (465.45,\; 458.18)
$$
