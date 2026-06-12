# 坐标系与变换符号

这个项目处理的是刚体坐标系之间的 6DoF 变换。先把符号约定写清楚，比后面多写一百行代码都值。

## 坐标系

- `W`: world frame。原型阶段可以把头环开机初始位姿作为世界原点。
- `H`: headset base frame。头环多摄像头刚体的中心坐标系。
- `C_i`: 第 `i` 个头环摄像头坐标系，例如 `C0`, `C1`, `C2`, `C3`。
- `T_i`: 手环上第 `i` 个 AprilTag 的局部坐标系。
- `B`: wristband rigid-body frame。手环几何中心，也就是希望代表手腕刚体的位置。
- `I`: IMU frame。IMU 芯片自己的坐标系。

## 变换约定

项目推荐使用：

```text
T_A_B
```

表示：

```text
把 B 坐标系里的点变换到 A 坐标系
```

也就是：

```text
p_A = T_A_B * p_B
```

你之前写的 `T_W<-H` 和 `T_W_H` 是同一个意思。

## `T_W<-H` 是什么

`T_W<-H` 表示：

```text
从 H 坐标系到 W 坐标系的变换
```

如果一个点 `p_H` 是在头环坐标系里表达的，那么：

```text
p_W = T_W<-H * p_H
```

等价写法：

```text
T_W<-H
T_W_H
pose of H in W
H-to-W transform
```

它就是头环在世界坐标系里的 6DoF 位姿。

这一项通常由 OpenVINS 估计：

```text
head cameras + head IMU -> OpenVINS -> T_W_H
```

## `T_H<-B` 是什么

`T_H<-B` 表示：

```text
从 B 坐标系到 H 坐标系的变换
```

如果一个点 `p_B` 是在手环坐标系里表达的，那么：

```text
p_H = T_H<-B * p_B
```

等价写法：

```text
T_H<-B
T_H_B
pose of B in H
B-to-H transform
```

它就是手环相对头环的 6DoF 位姿。

这一项通常由头环摄像头看到手环 AprilTag 后估计：

```text
head cameras + wrist AprilTags -> apriltag_ring_node -> T_H_B
```

## 最终手环世界坐标

最终目标是得到：

```text
T_W_B
```

也就是手环在世界坐标系里的位姿。

组合方式：

```text
T_W_B = T_W_H * T_H_B
```

直觉解释：

1. `T_W_H`: 头环在世界哪里。
2. `T_H_B`: 手环相对头环在哪里。
3. 两者相乘：手环在世界哪里。

## 从单个 AprilTag 推到手环中心

当第 `i` 个 camera `C_i` 看到第 `j` 个 tag `T_j` 时，PnP 可以得到：

```text
T_Ci_Tj
```

也就是 tag 在 camera 坐标系里的位姿。

再用 camera 外参：

```text
T_H_Ci
```

以及手环几何：

```text
T_Tj_B
```

就可以得到：

```text
T_H_B = T_H_Ci * T_Ci_Tj * T_Tj_B
```

## 四目时怎么做

四目不是简单地“每个相机算一个结果再平均”。更好的做法是：

```text
所有可见 camera
所有可见 tag
所有 2D corners
共同优化同一个 T_H_B
```

早期可以先做简化版：

1. 每个 camera 独立检测 AprilTag。
2. 每个可见 tag 独立估计 `T_H_B`。
3. 用 reprojection error / source_count / 时间连续性做筛选和加权。

稳定后再升级成 multi-camera joint PnP。

