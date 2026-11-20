"""Manual planar homography in cm; no camera intrinsics or hand-eye claims."""
import math


def number(v):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or abs(v) > 100000:
        raise ValueError('坐标必须是有限数字')
    return float(v)


def solve(a, b):
    rows = [list(r)+[v] for r, v in zip(a, b)]
    n = len(b)
    for j in range(n):
        p = max(range(j, n), key=lambda i: abs(rows[i][j]))
        if abs(rows[p][j]) < 1e-10:
            raise ValueError('标定点共线或分布退化，请选择覆盖画面的四角')
        rows[j], rows[p] = rows[p], rows[j]
        d = rows[j][j]
        rows[j] = [v/d for v in rows[j]]
        for i in range(n):
            if i != j:
                d = rows[i][j]
                rows[i] = [v-d*w for v, w in zip(rows[i], rows[j])]
    return [r[-1] for r in rows]


def project(h, pixel):
    u, v = [number(x)/1000 for x in pixel]
    d = h[6]*u+h[7]*v+1
    if abs(d) < 1e-8:
        raise ValueError('点接近平面投影奇异位置')
    return [(h[0]*u+h[1]*v+h[2])/d*100, (h[3]*u+h[4]*v+h[5])/d*100]


def fit(points):
    if not isinstance(points, list) or not 4 <= len(points) <= 30:
        raise ValueError('需要 4–30 组像素与平面坐标对应点')
    a, b = [], []
    for p in points:
        if len(p) != 4:
            raise ValueError('每组为 u、v、x_cm、y_cm')
        u, v, x, y = [number(t) for t in p]
        u, v, x, y = u/1000, v/1000, x/100, y/100
        a.extend([[u,v,1,0,0,0,-x*u,-x*v], [0,0,0,u,v,1,-y*u,-y*v]])
        b.extend([x,y])
    h = solve([[sum(r[i]*r[j] for r in a) for j in range(8)] for i in range(8)],
              [sum(r[i]*v for r, v in zip(a, b)) for i in range(8)])
    errors = [math.dist(project(h,p[:2]), p[2:]) for p in points]
    return dict(h=h, rms_cm=math.sqrt(sum(e*e for e in errors)/len(errors)), points=points,
                units='cm', model='manual planar homography; uncorrected distortion',
                warning='拟合残差不是独立精度；相机/场景移动后失效。物件顶面须单独标定或补偿高度。')
