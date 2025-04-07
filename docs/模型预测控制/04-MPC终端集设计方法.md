# MPC 终端集设计方法

---

## 基于线性不等式的迭代的终端集设计方法

该方法要求所有状态约束和控制约束都是线性的，且分别表示，MPC 问题表述如下：

$$
  \begin{aligned}
  \min_{u_{i|k}} J = & \sum_{i=1}^{N-1} \left\|  x(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  u(i-1|k) \right\|_{ R}^2 + \left\|  x(N|k) \right\|_{P}^2 \\
  \text{subj.to.} \quad &  x(0|k) =  x(k), \\
  & x(i|k) \in \mathcal X \subseteq \{x \in R^{n_x} |  H_x x \leq h_x \}, \  \forall i = 0, 1, \ldots, N, \\
  & u(i|k) \in \mathcal U \subseteq \{u \in R^{n_u} |  H_u u \leq h_u \}, \  \forall i = 0, 1, \ldots, N, \\
  & x(i+1|k) =  A  x(i|k) +  B  u(i|k), \  \forall i = 0, 1, \ldots, N-1, \\
  & x(N|k) \in \mathcal X_f \subset  \Omega.
  \end{aligned}
$$

选取由 LQR 算出的反馈增益$K_{LQR}$作为最后一步状态$x(N|k)$进入终端集后的控制策略。以$x(N|k)$和$u(N-1|k)$满足相应的状态约束、控制约束作为线性不等式迭代的初始条件。通过$K_{LQR}$更新$x(N+1|k)$和$u(N|k)$，得到新的线性不等式约束，并通过线性规划求状态量、控制量的最大值，观察是否满足所有约束。若满足就将当前所有的线性不等式围成的多面体作为终端集（实际上就是不停地取交集），否则继续迭代。

该方法伪代码如下：

!!! Example "**算法 1** 估计终端集$\mathcal{X}_f$（Maximal Admissible Positive Invariant Set）"

    **输入**: 系统矩阵$A,B$，约束矩阵$H_u, H_x$，约束向量$h_u, h_x$
    **输出**: 终端集$\mathcal{X}_f$

    ---

    **初始化阶段**

    1. 计算闭环系统矩阵：$A_K \leftarrow A - BK$
    2. 构造增广增益矩阵：$K_{\text{aug}} \leftarrow \begin{bmatrix} K_{\text{LQR}} \\ I \end{bmatrix} $
       其中$K_{\text{LQR}}$为LQR最优增益矩阵

    3. 定义增广约束：$H_{\text{aug}} \leftarrow \begin{bmatrix} H_u & 0 \\ 0 & H_x \end{bmatrix}, \quad h_{\text{aug}} \leftarrow \begin{bmatrix} h_u \\ h_x \end{bmatrix}$
    4. 初始化迭代参数：$t \leftarrow 0,\quad p_f \leftarrow 0,\  s \leftarrow \text{dim}(h), \ \mathcal{A}_{\text{LP}} \leftarrow \emptyset,\  \mathcal{B}_{\text{LP}} \leftarrow \emptyset$

    ---

    **迭代阶段**

    **while** $p_f = 0$ **do**:

    1. 更新线性规划约束：$\mathcal{A}_{\text{LP}} \leftarrow \begin{bmatrix} \mathcal{A}_{\text{LP}} \\ H_{\text{aug}}K_{\text{aug}}A_K^t \end{bmatrix}, \  \mathcal{B}_{\text{LP}} \leftarrow \begin{bmatrix} \mathcal{B}_{\text{LP}} \\ h_{\text{aug}} \end{bmatrix}$
    2. 计算下一时刻约束：$J_{\text{LP}} \leftarrow H_{\text{aug}}K_{\text{aug}}A_K^{t+1}$
    3. **for** $i = 1$ **to** $s$ **do**:
       $J_{next} \leftarrow \max_x \  J_{LP}[i] x \quad \text{subj.to.} \  \  A_{LP} x < b_{LP} $
       **if** $J_{\text{next}}^{(i)} < h^{(i)},\ \forall i \in \{1,...,s\}$ **then**:
       $p_f \leftarrow 1,\quad \mathcal{X}_f \leftarrow \{x \in \mathbb{R}^n | \mathcal{A}_{\text{LP}}x \preceq \mathcal{B}_{\text{LP}}\}$

    4. $t \leftarrow t + 1$


    **end while**

    **return** $\mathcal{X}_f$

## 基于最大椭球集/半定规划的终端集求解

对于离散线性系统

$$
  x(k+1) = A x(k) + B u(k),
$$

设计一个椭球集作为终端约束集合：

$$
  \mathcal X_f := \{ x|x^\top P x \leq 1 \} \subset \mathcal X
$$

使其既保证闭环稳定，又满足：

- 内部稳定性：$ x*k \in \mathcal{X}\_f \Rightarrow x*{k+1} \in \mathcal{X}_f$，
- 状态约束：$ H_x x \leq h_x $，
- 输入约束：$ H_u u \leq h_u $。

!!! note "" - 求终端集用到的$P$不需要与终端代价中的$P$相等。 - 寻找最大的终端集就是寻找最小的$\det P$，因此后续的推导过程中需要把离散 Lyapunov 方程变成不等式形式。 - $\det P$反映的是椭球集各轴综合缩放程度，因此$\det P$越小，椭球集越大。

### 内部稳定性

根据[离散Lyapunov方程](01-常用定理、矩阵.md#def:dis_Lya_func)，$P$需要满足如下条件：

$$
  (A + BK)^\top P (A +BK ) - P = -(Q + K^\top R K) \preceq 0.
$$

为了求最大终端集（椭球集），令

$$
  (A + BK)^\top P (A +BK ) - P \preceq -(Q + K^\top R K).
$$

为了将上述非线性约束转化为线性矩阵约束(LMI)，进行变量替换：

$$
\begin{aligned}
E &= P^{-1} \succ 0, \\
Y &= K E \Rightarrow K = Y E^{-1}.
\end{aligned}
$$

带入上式，并左乘右乘$ E $可得（注：$ E $是对称矩阵）：

$$
  E (A + B Y E^{-1})^\top E^{-1} (A + B Y E^{-1}) E - E + E(Q)E + Y^\top R Y,
$$

展开后化简为：

$$
  (AE + BY)^\top E^{-1} (AE + BY) - E + E Q E + Y^\top R Y \preceq 0.
$$

其中，单独看前两项，通过 shur 补引理可以转化为：

$$
\begin{bmatrix}
-E & (AE + BY)^\top \\
AE + BY & -E
\end{bmatrix}
=\begin{bmatrix}
-E & EA^\top + Y^\top B^\top \\
AE + BY & -E
\end{bmatrix}
\preceq 0.
$$

后两项可以分解为：

$$
E Q E + Y^\top R Y =
\begin{bmatrix}
E Q^{\frac 1 2} & Y^\top R^{\frac 1 2} \\
\end{bmatrix}
\begin{bmatrix}
Q^{\frac 1 2} E \\
R^{\frac 1 2} Y
\end{bmatrix}.
$$

进一步地，通过构造分块矩阵，可以得到公式(\ref{eq3:lya_func})的完整等价 LMI 如下：

$$
\begin{bmatrix}
-E & (AE + BY)^\top & Q^{1/2} E & Y^\top R^{1/2} \\
AE + BY & -E & 0 & 0 \\
E Q^{1/2} & 0 & -I & 0 \\
R^{1/2} Y & 0 & 0 & -I
\end{bmatrix} \preceq 0.
$$

### 约束处理

向量 $ H\_{x, i} x $ 的极大值出现在椭圆边界的切点（由椭圆半轴长公式，推导自拉格朗日极值）：

$$
  \max_{x \in \mathcal{X}_f} H_{x, i} x = \sqrt{ H_{x, i} E H_{x, i}^\top }.
$$

因此，状态约束的等价条件为（平方后应用 shur 补引理）：

$$
  H_{x, i} E H_{x, i}^\top \leq h_{x, i}^2 \Rightarrow
  \begin{bmatrix}
  h_{x, i}^2 & H_{x, i} E \\
  E H_{x, i}^\top & I
  \end{bmatrix} \succeq 0.
$$

同理，控制约束的等价 LMI 表述为：

$$
  \max_{x \in \mathcal{X}_f} H_{u, j} u = \sqrt{H_{u, j} Y E^{-1} E E^{-1} Y^\top H_{u, j}^\top} = \sqrt{H_{u, j} Y E^{-1} Y^\top H_{u, j}^\top},
$$

$$
H_{u, j} Y E^{-1} Y^\top H_{u, j}^\top \leq h_{u, j}^2  \Rightarrow
\begin{bmatrix}
h_{u, j}^2 & H_{u, j} Y \\
Y^\top H_{u, j}^\top & E
\end{bmatrix} \succeq 0.
$$

### 优化问题构造

综上所述，可以构造如下优化问题来求解终端集（稳定性中的$\preceq$变为$\succeq$）：

$$
    \begin{aligned}
    \min_{E, Y} \  & -\log \det E \\
    \text{subj.to.} \  & E \succeq 0\\
    &
    \begin{bmatrix}
    E & (AE + BY)^\top & Q^{1/2} E & Y^\top R^{1/2} \\
    * & * & 0 & 0 \\
    * & * & I & 0 \\
    * & * & * & I
    \end{bmatrix} \succeq 0, \\
    &
    \begin{bmatrix}
    h_{x, i}^2 & H_{x, i} E \\
    E H_{x, i}^\top & I
    \end{bmatrix} \succeq 0, \\
    &
    \begin{bmatrix}
    h_{u, j}^2 & H_{u, j} Y \\
    Y^\top H_{u, j}^\top & E
    \end{bmatrix} \succeq 0.
    \end{aligned}
$$

## 线性时变系统终端集求解（基于最大椭球集/半定规划）

考虑一个离散时间线性时变系统，其动态矩阵 $A_k(p)$ 和 $B_k(p)$ 是参数 $p$ 的函数：

$$
  x(k+1) = A_k(p)x(k) + B_k(p)u(k),
$$

其中$p \in \mathcal{P} \subseteq \mathbb{R}^d$ 是系统的时变参数，取值范围为 $\mathcal{P}$；
$A_k(p) \in \mathbb{R}^{n \times n}$ 和 $B_k(p) \in \mathbb{R}^{n \times m}$ 是参数化的系统矩阵。

系统的目标是设计一个固定的终端控制律 $u = Kx$ 和一个终端不变集 $\mathcal{X}_f$，使得：

- 闭环系统 $x(k+1) = (A_k(p) + B_k(p)K)x(k)$ 在参数 $p \in \mathcal{P}$ 的所有取值下是稳定的；
- 状态 $x(k)$ 和控制输入 $u(k)$ 满足给定的状态约束和控制约束；
- 终端不变集 $\mathcal{X}_f$ 是闭环系统的正不变集。

终端集的求解思路和线性时不变系统一样，但是需要满足所有参数下的状态矩阵和控制矩阵。
