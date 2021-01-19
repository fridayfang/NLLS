## 记`Eigen::Isometry3d`的奇怪用法
对于两个坐标系之间的关系，我们有相应的变换矩阵去描述:\
1. 2D-2D: 可以用$3\times3$的变换矩阵去计算坐标和向量在不同坐标系下的表示，变换矩阵的自由度是3
2. 3D-3D: 用$4\times 4$的变换矩阵计算点和向量在不同坐标系下的表示，自由度是6

而变换矩阵同时也可以表示位姿，变换矩阵的乘法也对应着相对位姿的累计。

我们将坐标系$C_1$到坐标系$C_2$的变换记做$T_2^1$,满足:
$$p_1 = T_2^1 p_2$$
其中$p_1$是某个向量或者点在坐标系$C_1$下的坐标表示，$p_2$是某个向量或者点在坐标系$C_2$下的坐标表示; 递推下去:
$$p_1 = T_2^1T_3^2...T_n^{n-1}p_n = T_n^1p_n$$
这样就描述了变换矩阵的乘法(算是左乘，变换矩阵在左边)和坐标变换的关系.

对于2D-2D的形式，下图所示的变换:
![](http://www.dengke.xyz/usr/uploads/2019/04/3840031707.jpg)
对应的变换矩阵:
$$
\begin{pmatrix}
\cos(\theta) & -\sin(\theta) & t_x \\
\sin(\theta)& \cos(\theta)& t_y \\
0 & 0  & 1
\end{pmatrix}
$$


在`Eigen::Isometry3d`是表示`3D-3D`的变换方式，下面的代码是正确的表示方式:
```cpp

int main(){
    Vector3d point(0,0,0);
    Isometry3d t = Isometry3d::Identity();
    t.prerotate(AngleAxisd(M_PI/2,Vector3d(0,0,1)));



    t.pretranslate(Vector3d(2,1,0));
    Vector3d p2 = t*point;

    cout<<p2<<endl;

}
```
但如果将`prerotate`和`pretranslate`交换顺序就会得到奇怪的表示，平移向量变成了旋转之后表示。而对于理解平移旋转的顺序来说，先平移后旋转对应的平移量才是原始坐标系的；而先旋转的后平移对应的平移量是旧坐标系下对新在新坐标系下的投影。这与`Isometry3d`的方式恰恰是相反的。

就我的常用方式来说，先平移后旋转是合理的，而对应的正确的代码则是先`prerotate`后`pretranslate`.
