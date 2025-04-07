# 有限时域线性有约束MPC稳定性推导

---

线性时不变系统的离散表达式满足：

<a id="eq:std_lti_st_func"></a>

\[
x(k+1) =  A  x(k) +  B  u(k)
\]

## 终端等式约束情况

终端等式指对预测时域的最后一个状态施加$x(N|k)=0$约束。

!!! info "命题"

    有限时域线性有约束MPC在设置终端等式约束$x(N|k)=0$后，在平衡点附近具有渐进稳定性。

    此时，有限时域线性时不变有约束模型预测控制问题描述如下：

    $$
    \begin{aligned}
    J = & \sum_{i=1}^{N} \left\|  x(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  u(i-1|k) \right\|_{ R}^2 \\
    \text{subj.to.} \quad &  x(0|k) =  x(k), \\
    & Gx(i|k) + Hu(i-1|k) \leq 1, \  \forall i = 0,1,\ldots, N,\\
    & x(i+1|k) =  A  x(i|k) +  B  u(i|k), \  \forall i = 0,1,\ldots, N-1,\\
    & x(N|k) = 0. 
    \end{aligned}
    $$

    其中，${ Q}$为半正定矩阵，${ R}$为正定矩阵。

## 终端不等式约束+终端代价情况

!!! info "命题"

    有限时域线性有约束MPC在选取合适的终端代价权重P，并设置终端不等式约束$x(N|k) \in \mathcal X_f \subset  \Omega$后（终端集需要具有控制不变性和约束兼容性），在平衡点附近具有渐进稳定性。

  

    此时，有限时域线性时不变有约束模型预测控制问题描述如下：

    $$
        \begin{aligned}
        J = & \sum_{i=1}^{N-1} \left\|  x(i|k) \right\|_{ Q}^2 + \sum_{i=1}^{N} \left\|  u(i-1|k) \right\|_{ R}^2 + \left\|  x(N|k) \right\|_{P}^2 \\
        \text{subj.to.} \quad &  x(0|k) =  x(k), \\
        & Gx(i|k) + Hu(i-1|k) \leq 1, \  \forall i = 0,1,\ldots, N,\\
        & x(i+1|k) =  A  x(i|k) +  B  u(i|k), \  \forall i = 0,1,\ldots, N-1,\\
        & x(N|k) \in \mathcal X_f \subset  \Omega. 
        \end{aligned}
    $$

    其中，${ Q}$为半正定矩阵，${ R}$为正定矩阵，$P$为$P - ( A -  B  K)^T  P ( A- B K) =  Q +  K^T  R  K$的解，$K$满足$|\text{eig}( A -  B K)| < 1$，$\Omega$为不变集。

!!! success "证明"

    因为$Gx(i|k) + Hu(i-1|k) \leq 1, \  \forall i = 0,1,\ldots, N$，
    故$x(N|k)$也满足状态约束。

    因此，设$\mathcal X$为状态约束集合，则有$x(N|k) \in \mathcal X_f \subset \Omega \subseteq \mathcal X$。

    由不变集的性质可知：

    $$
        x(i|k) \in \Omega \Rightarrow x(i+1|k) \in \Omega.
    $$

    所以有：$x(i|k) \in \mathcal X, \  \forall i = N+1,N+2,\ldots, \infty$。

    可以得到结论，只要$x(N|k)$在终端集内，往后的$x(i|k)$都自然满足状态约束。

    后续证明方法与无限时域线性有约束MPC一致。

终端不等式约束+终端代价情况下稳定性证明的核心是选取合适的终端集$X_f$。

## 稳定性的一般证明方法

其实MPC稳定性证明的核心就是选取包含终端代价的最优代价函数作为Lyapunov函数，同时通过设计终端约束/终端集为不变集，以此保证一旦系统进入终端集就可以通过终端控制律来构造可行解。

!!! abstract "定理"

    对于线性时不变系统

    $$
    x(k+1) = Ax(k) + Bu(k),
    $$

    假设：

    - 状态约束集合 $\mathcal{X}$ 和输入约束集合 $\mathcal{U}$ 是闭的凸集合，且包含原点；
    - 终端约束集合 $\mathcal{X}_f$ 满足以下条件：
      - $\mathcal{X}_f \subseteq \mathcal{X}$，且闭包；
      - $\mathcal{X}_f$ 是正不变的，即如果 $x_{N|k} \in \mathcal{X}_f$，则在终端控制律 $u = Kx$ 下，后续状态 $x(k) \in \mathcal{X}_f$；
      - 终端控制律 $u = Kx$ 使得闭环系统 $x(k+1) = (A + BK)x(k)$ 渐近稳定；
    - 终端权重矩阵 $P$ 满足离散Lyapunov方程：
  

    $$
    \begin{aligned}
        &  P - ( A -  B  K)^T  P ( A- B K) =  Q +  K^T  R  K, \\
        & |\text{eig}( A -  B K)| < 1
    \end{aligned}
    $$

      且 $P \succeq 0$；

    - 权重矩阵 $Q \succeq 0$、$R \succ 0$。
      则在上述条件下，采用以下MPC控制律：

    $$
    u(k) = u_{0|k}^*,
    $$

      其中 $u_{0|k}^*$ 是优化问题

    $$
    \begin{aligned}
    \min_{u_{i|k}} J &= \sum_{i=0}^{N-1} \left( \|x_{i|k}\|_Q^2 + \|u_{i|k}\|_R^2 \right) + \|x_{N|k}\|_P^2 \\
    \text{subj.to.} \quad &  x(0|k) =  x(k), \\
    & x_{i+1|k} = Ax_{i|k} + Bu_{i|k}, \  \forall i = 0,1,\ldots, N-1,\\
    & x_{i|k} \in \mathcal{X},  \  \forall i = 0,1,\ldots, N,\\
    & u_{i|k} \in \mathcal{U},  \  \forall i = 0,1,\ldots, N,\\
    & x_{N|k} \in \mathcal{X}_f,
    \end{aligned}
    $$

      的最优解，所得到的闭环系统是渐近稳定的。

!!! success "证明"

    定义 Lyapunov 函数 $V(x(k))$ 为优化问题的最优目标值：

    $$
    V(x(k)) = \sum_{i=0}^{N-1} \left( \|x_{i|k}^*\|_Q^2 + \|u_{i|k}^*\|_R^2 \right) + \|x_{N|k}^*\|_P^2,
    $$

    其中：

    - $x_{i|k}^*$ 和 $u_{i|k}^*$ 是以当前状态 $x(k)$ 为初始状态的最优预测序列；
    - $x_{0|k}^* = x(k)$；
    - $\|x\|_Q^2 = x^\top Q x$ 表示加权的 $\ell_2$ 范数平方。
    
    首先，需要证明 $V(x(k))$ 是一个 Lyapunov 函数，即：

    - $V(x(k)) \geq 0$，且 $V(x(k)) = 0 \iff x(k) = 0$；
    - $V(x(k+1)) - V(x(k)) \leq -\alpha \|x(k)\|^2$，其中 $\alpha > 0$。
    由于 $Q \succeq 0$、$R \succ 0$、$P \succeq 0$，可以证明：

    $$
    V(x(k)) \geq 0, \quad \forall x(k) \in \mathbb{R}^n.
    $$

    此外，如果 $x(k) = 0$，则最优状态序列和控制序列为零（即 $x_{i|k}^* = 0, u_{i|k}^* = 0$），从而：
    
    $$
    V(x(k)) = 0.
    $$

    反之，如果 $V(x(k)) = 0$，则 $x_{i|k}^* = 0, u_{i|k}^* = 0$，从而 $x(k) = 0$。

    因此，$V(x(k))$ 是正定函数。

    然后，需要证明Lyapunov函数的下降性。

    假设在时刻 $k$，优化问题的最优解为：

    - 最优控制输入序列：$\bm{u}^* = \{u_{0|k}^*, u_{1|k}^*, \dots, u_{N-1|k}^*\}$；
    - 最优状态序列：$\bm{x}^* = \{x_{0|k}^*, x_{1|k}^*, \dots, x_{N|k}^*\}$，其中 $x_{0|k}^* = x(k)$。
    应用第一个控制输入 $u(k) = u_{0|k}^*$ 后，系统进入下一个状态：
    
    $$
    x(k+1) = Ax(k) + Bu_{0|k}^*.
    $$

    在时刻 $k+1$，构造一个可行控制序列：

    $$
    \tilde{\bm{u}} = \{u_{1|k}^*, u_{2|k}^*, \dots, u_{N-1|k}^*, Kx_{N|k}^*\},
    $$
    
    对应的状态序列为：
    
    $$
    \tilde{\bm{x}} = \{x_{0|k+1}, x_{1|k+1}, \dots, x_{N|k+1}\},
    $$
    

    其中：

    
    $$
    \begin{aligned}
        x_{0|k+1} &= x(k+1) = x_{1|k}^*, \\
        x_{i+1|k+1} &= Ax_{i|k+1} + Bu_{i|k+1}, \quad \forall i = 0, 1, \dots, N-1.
    \end{aligned}
    $$

    
    $$
    V(x(k+1)) \leq \sum_{i=0}^{N-1} \left( \|x_{i|k+1}\|_Q^2 + \|u_{i|k+1}\|_R^2 \right) + \|x_{N|k+1}\|_P^2.
    $$

    将 $\tilde{\bm{u}}$ 和 $\tilde{\bm{x}}$ 的具体形式代入，可以得到：
    
    $$
    V(x(k+1)) \leq \sum_{i=1}^{N-1} \left( \|x_{i|k}\|_Q^2 + \|u_{i|k}\|_R^2 \right) + \|x_{N|k}\|_Q^2 + \|Kx_{N|k}^*\|_R^2 + \|(A+BK)x_{N|k}^*\|_P^2.
    $$

    利用离散Lyapunov函数的性质，可以证明：
    
    $$
    \|x_{N|k}\|_Q^2 + \|Kx_{N|k}^*\|_R^2 + \|(A+BK)x_{N|k}^*\|_P^2 \leq \|x_{N|k}\|_P^2.
    $$

    因此，有：
    
    $$
    V(x(k+1)) - V(x(k)) \leq -\|x_{0|k}\|_Q^2 - \|u_{0|k}\|_R^2.
    $$

    令 $\alpha = \min\{\lambda_{\min}(Q), \lambda_{\min}(R)\}$，则：
    
    $$
    V(x(k+1)) - V(x(k)) \leq -\alpha \|x(k)\|^2.
    $$

    综上，根据Lyapunov稳定性理论，$V(x(k))$是正定函数，且沿闭环系统轨迹单调递减，因此闭环系统是渐近稳定的。
