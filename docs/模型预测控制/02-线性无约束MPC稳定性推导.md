# 线性无约束MPC稳定性推导

---

线性时不变系统的离散表达式满足：

<a id="eq:std_lti_st_func"></a>

\[
x(k+1) =  A  x(k) +  B  u(k)
\]

## 无限时域情况

!!! info "命题"

    无限时域线性无约束MPC在平衡点附近具有渐进稳定性，且等价于LQR。
    此时，无限时域线性时不变无约束模型预测控制问题描述如下：

    $$
    \begin{aligned}
    J = & \min_{U(k)} \sum_{i=1}^{\infty} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) \\
    \text{subj.to.} \quad &  x(0|k) =  x(k), \\
    &  x(i+1|k) =  A  x(i|k) +  B  u(i|k), \  \forall i = 0, 1, \ldots, N-1. 
    \end{aligned}
    $$

    其中，${ Q}$为半正定矩阵，${ R}$为正定矩阵。

!!! success "证明"

    将当前时刻$k$计算得到的最优控制序列直接应用到下一时刻$k$：

    \[
    u(i|k+1) = u^*(i+1|k).
    \]

    可以得到"State shift"：

    \[
        x(i|k+1) = x^*(i+1|k).
    \]

    选取最优代价函数$J^*$作为Lyapunov函数，可以证明：

    \[
        J^{*}(k+1)-J^{*}(k) \leq J(k+1)-J^{*}(k) = -\|x(1|k)\|_{Q}^{2}-\|u(0|k)\|_{R}^{2} < 0
    \]

    由于是无限时域，再加上“State shift”，$J(k+1)-J^{*}(k)$仅第一项存在差异。

    综上，由于$J^{*}$本身正定，而$J^{*}(k+1)-J^{*}(k)$负定，所以根据[Lyapunov稳定性定理](01-常用定理、矩阵.md#theorem:lya_stab)可知无限时域线性无约束MPC在平衡点附近是渐进稳定的。

## 有限时域情况

!!! info "命题"

    有限时域线性无约束MPC在选取合适的终端代价权重P后，在平衡点附近具有渐进稳定性。
    其中，$P$是[离散Lyapunov方程](01-常用定理、矩阵.md#def:dis_Lya_func)的解。

    
    此时，有限时域线性时不变无约束模型预测控制问题描述如下：
    
    \[
        \begin{align}
        J = & \sum_{i=1}^{N-1} \left\|  x(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  u(i-1|k) \right\|_{ R}^2 + \left\|  x(N|k) \right\|_{P}^2 \\
        \text{subj.to.} \quad &  x(0|k) =  x(k), \\
        &  x(i+1|k) =  A  x(i|k) +  B  u(i|k), \  \forall i = 0,1,\ldots, N-1. 
        \end{align}
    \]

    其中，${ Q}$为半正定矩阵，${ R}$为正定矩阵，$P$为$P - ( A -  B  K)^T  P ( A- B K) =  Q +  K^T  R  K$的解，$K$满足$|\text{eig}( A -  B K)| < 1$。

!!! success "证明"

    核心是证明有限时域无约束MPC代价函数等价于无限时域MPC的代价函数。

  

    根据[离散Lyapunov方程](01-常用定理、矩阵.md#def:dis_Lya_func)可知：

    $$
        \begin{align}
        x^T[P-(A-BK)^TP(A-BK)]x &= x^T[Q+K^TRK]x, \\
        x^TPx - x^T(A-BK)^TP(A-BK)x &= x^TQx + x^TK^TRKx.
        \end{align}
    $$

    带入[公式](#eq:std_lti_st_func)可知：

    $$
        x(k)^T P x(k) - x^T(k+1)^T P x(k+1) = x^T(k) Q x(k) + u^T(k) R u(k).
    $$

    将$x(k)$改写为$x(i|k)$，$x(k+1)$改写为$x(i+1|k)$，并将上式从$i=0$导$i=\infty$求和可以得到：

    $$
        \begin{aligned}
        & x(0|k)^T P x(0|k) -\underbrace{ x^T(\infty|k)^T P x(\infty|k)}_{\approx 0} = \sum_{i = 0}^{i = \infty}(x^T(i|k) Q x(i|k) + u^T(i|k) R u(i|k)) \\
        & x(0|k)^T P x(0|k) = x^T(0|k) Q x(0|k) + \sum_{i = 1}^{i = \infty}(x^T(i|k) Q x(i|k) + u^T(i-1|k) R u(i-1|k)) \\
        & x(0|k)^T P x(0|k) - x^T(0|k) Q x(0|k) = \sum_{i = 1}^{i = \infty}(x^T(i|k) Q x(i|k) + u^T(i-1|k) R u(i-1|k)) \\
        & \xrightarrow{x(0|k) = x(k)} x(k)^T P x(k) - x^T(k) Q x(k) = J(k)
        \end{aligned}
    $$

    代价函数$J(k)$还可以通过分段，写成：

    $$
        \begin{aligned}
        J(k) &= \sum_{i=1}^{N-1} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) + \sum_{i=N}^{\infty} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) \\
        &= \sum_{i=1}^{N-1} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) + \sum_{i=N}^{\infty} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) + \left\| u(N-1|k) \right\|_{ R}^2 \\
        &=\sum_{i=1}^{N-1} (\left\|  x(i|k) \right\|_{ Q}^2 + \left\|  u(i-1|k) \right\|_{ R}^2) + \left\|  x(N|k) \right\|_{P}^2 + \left\| u(N-1|k) \right\|_{ R}^2 \\
        &=\sum_{i=1}^{N-1} \left\|  x(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  u(i-1|k) \right\|_{ R}^2 + \left\|  x(N|k) \right\|_{P}^2 \\
        &= X(k)^T \mathscr{Q} X(k) + U^T(k) \mathscr{R} U(k) .
        \end{aligned}
    $$

    其中，

    $$
        \begin{aligned}
        X(k) &= [x(1|k), x(2|k), \ldots, x(N|k)]^T, \\
        U(k) &= [u(0|k), x(1|k), \ldots, u(N-1|k)]^T, \\
        \mathscr{Q} &= \text{diag}[\underbrace{Q, Q, \ldots, Q}_{N-1},P],\\
        \mathscr{R} &= \text{diag}[\underbrace{R, R, \ldots, R, R}_{N}].
        \end{aligned}
    $$

    当$k<N$时，都采用最优控制律，其余时候采用负反馈控制律($|\text{eig}( A -  B K)| < 1$)，即：

    $$
        \begin{aligned}
        u(i|k+1) &= u^*(i+1|k) \quad \text{for all } i = 0,1,\ldots,N-1， \\
        u(i|k+1) &= -Kx(i|k+1) \quad \text{for all } i = N,N+1,\ldots.
        \end{aligned}
    $$
    
    后续证明方法与无限时域线性无约束MPC一致。
