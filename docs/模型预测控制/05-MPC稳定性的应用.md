# MPC稳定性的应用
---

## Robust MPC/Tube-based MPC

### 问题描述

Robust MPC主要用于处理预测模型不准确，且不准确性有界时的问题。

真实模型(Real model)表示为：

$$
\begin{aligned}
x(k+1) &= A x(k) + B u(k) + D w(k)， \\
w(k) &\in \mathcal W,\  \forall i = 0,1,\ldots, N-1.\\
\end{aligned}
$$

标称模型(Norminal model)表示为：

$$
  z(k+1) = A z(k) + B v(k)，
$$
其中，$z(0) = x(0)$。

Robust MPC的基本结构是“标称MPC+线性反馈控制”。设反馈控制律为：$-K_{\mathcal E}(x(k) - z(k))$，可以得到误差模型：

$$
\begin{aligned}
e(k+1) &= (A - B K_{\mathcal E}) e(k) + D w(k)， \\
&= A_K e(k) + D w(k),
\end{aligned}
$$
其中，$e(k) = x(k) - z(k), \  u(k) = v(k) - K_{\mathcal E}(x(k) - z(k))$，$K_{\mathcal E}$满足$|\text{eig}( A -  B K)| < 1$。

### 稳定性修正

由于初始时刻会直接用采集到的真实状态作为标称模型的输入，即$x(0) - z(0)$，因此根据误差模型的递推关系可以得到：

$$
\begin{aligned}
e(0) &= x(0) - z(0) = 0, \\
e(1) &= A_K e(0) + D w(0) = D w(0), \\
e(2) &= A_K e(1) + D w(1) = A_K D w(0) + D w(1), \\
e(3) &= A_K e(2) + D w(2)= A_K^2 D w(0) + A_K D w(1) + D w(2), \\
&\vdots \\
e(k) &= \cdots = \sum_{i=1}^{k} A_K^{i-1} D w_{k-j}.
\end{aligned}
$$

又因为$|\text{eig}( A -  B K)| < 1$，且$w_k$有界($w(i|k) \in \mathcal W$)，故：

$$
  \begin{aligned}
    e(k) &= \sum_{i=1}^{k} A_K^{i-1} D w_{i-1} \in \sum_{i=1}^{k} A_K^{i-1} D \mathcal{W} \subset \sum_{i=1}^{\infty} A_K^{i-1} D \mathcal{W}, \\
    \mathcal E &:= \sum_{i=1}^{\infty} A_K^{i-1} D \mathcal{W} \subset \Omega.
  \end{aligned}
$$

由于$\mathcal E$是有限集，因此误差$e(k)$有界，并且$\mathcal E$是$e(k)$的不变集。

因为$e \in \mathcal E$，所以$K_{\mathcal E} e \in K_{\mathcal E} \mathcal E$。为了保证$x \in \mathcal X, u \in \mathcal U$，闭环标称系统需要满足：

$$
  \begin{aligned}
    z \in \mathcal Z = \mathcal X \ominus \mathcal E,\\
    v \in \mathcal V = \mathcal U \ominus K_{\mathcal E} \mathcal E.
  \end{aligned}
$$

同时，为了保证标称系统的稳定性，标称系统的终端集需要是不变集：

$$
  z(N|k) \in \mathcal Z_f \subset \Omega.
$$

!!! note ""
    $Z_f$的求法和正常终端集一致。


!!! note ""
    由于标称模型的终端状态$z_N$和误差模型的状态$z_k$都在不变集内，因此真实模型的终端状态$x_N$也在不变集($\mathcal Z_f \oplus \mathcal E$)内，因此可以保证稳定性。


综上所述，Robust MPC问题可以表示为：

$$
  \begin{aligned}
  \min_{u(i|k)} J = & \sum_{i=1}^{N-1} \left\|  z(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  v(i-1|k) \right\|_{ R}^2 + \left\|  z(N|k) \right\|_{P}^2 \\
  \text{subj.to.} \quad &  z(0) =  x(0), \\
  & z(i|k) \in \mathcal X \ominus \mathcal E, \  \forall i = 1,2,\ldots, N,\\
  & v(i|k) \in \mathcal U \ominus K_{\mathcal E} \mathcal E, \  \forall i = 0,1,\ldots, N-1,\\
  & z(i+1|k) =  A  z(i|k) +  B v(i|k), \  \forall i = 0,1,\ldots, N-1,\\
  & z(N|k) \in \mathcal Z_f \subset  \Omega. 
  \end{aligned}
$$

!!! note ""
    $K_{\mathcal E}$可以直接用LQR来求，$\mathcal E$可以通过某些手段求解。

## 预测安全滤波器 (Predictive safety filter, PSF)

PSF的目的是对已有的控制量$u_{\mathcal{L}}$进行修正，计算新的控制量$u_{\mathcal{S}}$，使得被控系统能够在未来一段时间$N$内始终满足控制量、状态量约束，并且状态量能够收敛到安全终端集$\mathcal S_f$内。

PSF和MPC稳定性修正的最本质区别是PSF不保证被控系统的稳定性，只保证状态量处于安全约束内，所以PSF的代价函数和MPC差别很大，并且不需要终端代价。

### 标准线性模型PSF

标准线性模型的PSF的MPC问题表示如下：

<a id="eq:norm_PSF"></a>

$$
  \begin{aligned}
  \min_{u_{i|k}} J = & \left\|  u_{\mathcal{L}} - u_{0|k} \right\|_W^2\\
  \text{subj.to.} \quad &  x_{0|k} =  x(k), \\
  & x_{i|k} \in \mathcal X \subset \{x \in R^{n_x} |  H_x x \leq h_x \}, \  \forall i = 1,2,\ldots, N,\\
  & u_{i|k} \in \mathcal U \subset \{u \in R^{n_u} |  H_u u \leq h_u \}, \  \forall i = 0,1,\ldots, N-1,\\
  & x_{i+1|k} =  A  x_{i|k} +  B  u_{i|k}, \  \forall i = 0,1,\ldots, N-1,\\
  & x_{N|k} \in \mathcal S_f \subset  \Omega. 
  \end{aligned}
$$

终端安全集$\mathcal S_f$具有如下性质：存在一个控制律 \( u_f: \mathcal S_f \to \mathcal U \)，以及一个对应的正不变集 \( \mathcal S_f \subseteq \mathcal{X} \)，使得对于所有 \( x \in \mathcal S_f \)，都有 \( u_f(x) \in \mathcal U \) 且 \( f(x, u_f(x)) \in \mathcal S_f \)。

每次求解[优化问题](#eq:norm_PSF)时，

- 若有可行解，则令$u_{\mathcal{S}} = u_{0|k}^*$，并定义$\bar k = k$，储存\( \bar k \)时刻的N个最优控制序列；
- 若无可行解，
    - 如果$ k - \bar k \leq N-1$，令 $u_{\mathcal{S}} = u_{k- \bar k|\bar k}^*$；
    - 如果$ k - \bar k > N-1$，令 $u_{\mathcal{S}} = u_f(x(k))^*$。


!!! note ""
    $u_f$和$\mathcal S_f$的求法和稳定性证明中的终端集一致。

### 线性误差模型PSF

线性误差模型PSF的表达式与Tube-MPC基本一致，其核心都是需要令标称模型的约束集等于“真实的约束集合”$\ominus$“模型误差的不变集”。标称模型终端控制律变为：存在一个控制律 \( v_f: \mathcal S_f \ominus \mathcal{E} \to \mathcal U \ominus K_{\mathcal{E}} \mathcal{E} \)，以及一个对应的正不变集 \( \mathcal S_f \subseteq \mathcal{X} \)，使得对于所有 \( z \in \mathcal S_f \ominus \mathcal{E} \)，都有 \( v_f(x) \in \mathcal U \ominus K_{\mathcal{E}} \mathcal{E} \) 且 \( f(x, v_f(x)) \in \mathcal S_f \ominus \mathcal{E} \)。

线性误差模型PSF表示形式如下：

$$
  \begin{aligned}
  \min_{v_{i|k}} \quad  & J = \left\|  u_{\mathcal{L}} - u_{0|k} \right\|_W^2\\
  \text{subj.to.} \quad &  x_{0|k} =  x(k), \\
  & z_{i|k} \in \mathcal X \ominus \mathcal{E}, \  \forall i = 1,2,\ldots, N,\\
  & v_{i|k} \in \mathcal U \ominus K_{\mathcal{E}} \mathcal{E}, \  \forall i = 0,1,\ldots, N-1,\\
  & z_{i+1|k} =  A  z_{i|k} +  B  v_{i|k}, \  \forall i = 0,1,\ldots, N-1,\\
  & u_{0|k} = v_{0|k} -K_{\mathcal{E}}\left(x(k) - z_{k- \bar k|\bar{k}}\right),\\
  & z_{N|k} \in \mathcal Z_f =  \mathcal S_f \ominus \mathcal{E} \subset  \Omega. 
  \end{aligned}
$$

每次求解上述优化问题时，

- 若有可行解，则令$u_{\mathcal{S}} = u_{0|k}^*$，并定义 $\bar k = k$，储存\( \bar k \)时刻的N个最优标称控制序列；
- 若无可行解，
    - 如果$ k - \bar k \leq N-1$，令 $u_{\mathcal{S}} = v_{k- \bar k|\bar k}^* - K_{\mathcal{E}}\left(x(k) - z_{k- \bar k|\bar k}\right)$；
    - 如果$ k - \bar k > N-1$，令 $u_{\mathcal{S}} =  - K_{\mathcal{E}}\left(x(k) - z_{k- N|\bar k}\right)$。