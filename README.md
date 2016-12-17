# Two-space-linear-collision-get-minimum-distance-and-nearest-point
 This project is based on the known two linear equations in three-dimensional space,  to get the minimum distance of two straight lines and the distance between two straight lines closest point。
 一、理论：
已知空间中两线段，如果它们无限变粗，判断是否相交。（主要讨论不在同一平面的情况）
线段AB，线段CD
问题的关键是求出这两条任意直线之间的最短距离，以及在这个距离上的两线最接近点坐标，判断该点是否在线段AB和线段CD上。
首先将直线方程化为对称式，得到其方向向量n1=（a1,b1,c1),n2=(a2,b2,c2).
再将两向量叉乘得到其公垂向量N=（x,y,z）,在两直线上分别选取点A,B(任意)，得到向量AB，
求向量AB在向量N方向的投影即为两异面直线间的距离了（就是最短距离了）。
最短距离的求法：d=|向量N*向量AB|/|向量N|（上面是两向量的数量积，下面是取模）。
设交点为C,D，带入公垂线N的对称式中，又因为C,D两点分别满足一开始的直线方程，所以得到关于C（或D）的两个连等方程，分别解出来就好了！

