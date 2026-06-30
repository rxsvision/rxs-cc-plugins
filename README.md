# RXS Mesh Tools

> 点云重建与数模切割命令行工具集 -- 用于三维点云曲面重建（PCD -> STL -> STEP）及 STL 数模按截面切割采样（STL -> PCD）。

## 工具概览

本仓库包含两个独立的命令行工具：

| 工具 | 功能 | 数据流 |
|------|------|--------|
| **mesh2cad** | 点云曲面重建 | PCD 点云 -> 法线估计 -> Poisson 重建 -> STL 网格 -> STEP 实体 |
| **meshSlice** | 数模截面切割采样 | STL 数模 -> 按 Z/X 截面切割 -> 三角面片采样 -> 滤波去噪 -> PCD 点云 |

---

## mesh2cad

将 PCD 点云文件经过 Poisson 曲面重建生成 STL 三角网格，再通过 OpenCASCADE 转换为 STEP 实体模型。

### 处理流程

1. **读取点云** -- 加载 PCD 文件（`pcl::PointXYZ`）
2. **法线估计** -- 使用 KdTree 近邻搜索，在指定半径内估计法向量（`pcl::NormalEstimation`）
3. **Poisson 重建** -- 基于带法线的点云进行泊松曲面重建（深度参数 `depth=9`）
4. **网格后处理** -- VTK 三角化滤波 + 法线计算
5. **导出 STL** -- 以二进制格式写入 STL 文件
6. **导出 STEP** -- 通过 OpenCASCADE 的 `StlAPI_Reader` 读取 STL，`STEPControl_Writer` 导出 STEP

### 构建依赖

mesh2cad 需要以下库：

- **PCL** (common, io, surface, features, visualization, kdtree, filters)
- **VTK** (通过 PCL 间接依赖；直接使用 `vtkSTLWriter`, `vtkTriangleFilter`, `vtkPolyDataNormals` 等)
- **CGAL** (Delaunay 三角剖分模块，用于备用重建方案)
- **OpenCASCADE** (`TKSTEP`, `TKSTL`, `TKBRep`, `TKBO`, `TKernel` 等)
- **Eigen3**
- **Boost** (PCL 依赖)

### 使用方式

当前 `main.cpp` 中使用硬编码路径，可按需修改输入输出文件名：

```cpp
// 输入：PCD 点云文件
pcl::io::loadPCDFile<pcl::PointXYZ>("small.pcd", *cloud);

// 法线估计半径参数（构造函数第二个参数）
pcd2stl ps(cloud, 2);  // radius = 2

// 输出：STL 网格文件
ps.saveSTL("output_mesh.stl");

// 输出：STEP 实体文件
writer.Write("output.step");
```

### 核心源文件

| 文件 | 说明 |
|------|------|
| `main.cpp` | 主入口，串联 PCD -> STL -> STEP 流程 |
| `pcd2stl.h` / `pcd2stl.cpp` | 法线估计 + Poisson 重建 + STL 导出封装类 |
| `delaunay.h` / `delaunay.cpp` | CGAL Delaunay 2D/3D 三角剖分 + MLS 平滑（备用方案） |
| `czxDependence/czxTool.h` / `.cpp` | 通用工具库：日志、计时器、配置文件读取、点云可视化 |

---

## meshSlice

读取 STL 数模文件，按照配置的 Z 截面和 X 截面进行切割，对切割后的三角面片进行面积加权随机采样生成点云，经滤波后输出 PCD 文件。

### 处理流程

1. **读取配置** -- 解析 `meshSlice.czx` 配置文件
2. **加载 STL** -- 通过 VTK 的 `vtkSTLReader` 读取数模
3. **截面切割** -- 使用 `vtkClipPolyData` + `vtkPlane` 沿 Z 轴或 X 轴方向进行双平面裁剪（厚度 +/- 0.02）
4. **点云采样** -- 对裁剪后的三角面片，按面积 / (分辨率^2) 确定每面采样数，使用重心坐标随机采样（分辨率 `0.005`）
5. **坐标变换** -- Z 截面切割结果投影到 XY 平面（Z 清零）；X 截面切割结果交换 XZ 轴后投影
6. **排序与滤波** -- 按 X 坐标排序后，执行 `filterShelterAndDense` 去除遮挡点和过密点
7. **输出 PCD** -- 以二进制 PCD 格式保存，文件命名 `{序号}x.pcd`（Z 截面）或 `{序号}y.pcd`（X 截面）

### 配置文件格式

工具运行前需在工作目录下放置 `meshSlice.czx` 配置文件，格式为 `key=value`：

```ini
path=D:\models\workpiece.stl
z=154.030609,95.279526
x=-49.866207,-14.974101
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `path` | string | STL 数模文件的绝对路径 |
| `z` | float, 逗号分隔 | Z 轴截面位置列表。每个 Z 值对应一次水平切割，输出 `{序号}x.pcd`（X 方向轮廓点云） |
| `x` | float, 逗号分隔 | X 轴截面位置列表。每个 X 值对应一次纵向切割，输出 `{序号}y.pcd`（Y 方向轮廓点云） |

### 输出说明

- 输出目录为程序运行时按当前日期自动创建的文件夹
- Z 截面输出文件命名为 `1x.pcd`, `2x.pcd`, ...（按配置中的顺序编号）
- X 截面输出文件接续编号，如 `3y.pcd`, `4y.pcd`, ...

### 构建依赖

meshSlice 需要以下库：

- **PCL** (common, io)
- **VTK** (直接使用 `vtkSTLReader`, `vtkClipPolyData`, `vtkPlane` 等；通常随 PCL 安装)
- **Eigen3** (通过 PCL 间接依赖)
- **Boost** (PCL 依赖)

### 核心源文件

| 文件 | 说明 |
|------|------|
| `源.cpp` | 主入口，包含切割采样、滤波、配置解析全部逻辑 |
| `meshSlice.czx` | 示例配置文件 |
| `../mesh2cad/czxDependence/czxTool.h` | 共享工具库（配置文件读取、类型定义等） |

---

## 依赖版本要求

| 依赖 | 建议版本 | 说明 |
|------|----------|------|
| CMake | >= 3.16 | 构建系统 |
| C++ 编译器 | C++17 | MSVC 2019+ / GCC 9+ / Clang 10+ |
| PCL | 1.12 -- 1.14 | Point Cloud Library；需编译 common, io, surface, features, visualization, kdtree, filters 模块 |
| VTK | 9.1 -- 9.3 | Visualization Toolkit；PCL 编译时需开启 VTK 支持 |
| CGAL | 5.4 -- 5.6 | Computational Geometry Algorithms Library（仅 mesh2cad 使用） |
| OpenCASCADE | 7.7 -- 7.8 | OCCT 开源版本（仅 mesh2cad 使用） |
| Eigen3 | 3.4+ | 线性代数库 |
| Boost | 1.78+ | PCL 和工具库的间接依赖 |

---

## 构建指南

### 1. 安装依赖（Windows / vcpkg）

推荐使用 [vcpkg](https://github.com/microsoft/vcpkg) 管理 C++ 依赖：

```bash
# 安装 vcpkg
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
.\bootstrap-vcpkg.bat

# 安装核心依赖（PCL 会自动拉取 VTK、Eigen3、Boost 等）
.\vcpkg install pcl:x64-windows

# 安装 CGAL（仅 mesh2cad 需要）
.\vcpkg install cgal:x64-windows

# 安装 OpenCASCADE（仅 mesh2cad 需要）
.\vcpkg install opencascade:x64-windows
```

### 2. 安装依赖（Linux / apt + 源码编译）

```bash
# 系统包
sudo apt update
sudo apt install -y cmake build-essential libeigen3-dev libboost-all-dev

# PCL（含 VTK）
sudo apt install -y libpcl-dev

# CGAL
sudo apt install -y libcgal-dev

# OpenCASCADE（需从源码或使用 OCP 预编译包）
# 参考: https://dev.opencascade.org/doc/overview/html/index.html
```

### 3. CMake 构建

```bash
# 在仓库根目录
cmake -B build -S . \
  -DCMAKE_TOOLCHAIN_FILE=<vcpkg-root>/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release

# 编译
cmake --build build --config Release

# 可选：只构建某一个工具
cmake -B build -S . \
  -DRXS_BUILD_MESH2CAD=ON \
  -DRXS_BUILD_MESHSLICE=OFF
```

### 4. MSVC / Visual Studio 构建

项目包含 `.sln` 和 `.vcxproj` 文件，可直接在 Visual Studio 中打开：

- `mesh2cad/mesh2cad.sln` -- mesh2cad 工程
- `meshSlice/meshSlice.sln` -- meshSlice 工程

需确保 PCL、VTK、CGAL、OpenCASCADE 的路径已配置在项目属性中（或使用 vcpkg 集成）。

### CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `RXS_BUILD_MESH2CAD` | `ON` | 是否构建 mesh2cad |
| `RXS_BUILD_MESHSLICE` | `ON` | 是否构建 meshSlice |

---

## 项目结构

```
rxs-cc-plugins/
├── CMakeLists.txt              # 顶层 CMake（C++17，子目录开关）
├── LICENSE                     # BSL 1.1 许可证
├── SECURITY.md                 # 安全策略
├── README.md                   # 本文件
│
├── mesh2cad/                   # 点云 -> STL -> STEP 重建工具
│   ├── CMakeLists.txt
│   ├── main.cpp                # 主入口
│   ├── pcd2stl.h / .cpp        # Poisson 重建 + STL 导出
│   ├── delaunay.h / .cpp       # CGAL Delaunay 三角剖分 + MLS 平滑
│   ├── czxDependence/          # 共享工具库
│   │   ├── czxTool.h / .cpp    # 日志、计时器、配置读取、可视化
│   │   └── pcl_release.props   # VS 属性表
│   ├── mesh2cad.sln            # Visual Studio 解决方案
│   └── mesh2cad.vcxproj        # VS 项目文件
│
├── meshSlice/                  # STL 数模截面切割采样工具
│   ├── CMakeLists.txt
│   ├── 源.cpp                  # 主入口（切割、采样、滤波）
│   ├── meshSlice.czx           # 示例配置文件
│   ├── meshSlice.sln           # Visual Studio 解决方案
│   ├── meshSlice.vcxproj       # VS 项目文件
│   ├── 配置说明.txt
│   └── 数模切割配置文件说明.txt
│
├── docs/                       # 开发文档
│   ├── 变形度SDK.docx
│   └── 点云到点云的距离SDK.docx
│
└── .github/
    ├── workflows/
    │   ├── ci.yml              # PR / push CI（编码检查、CMake lint、clang-format）
    │   ├── codeql.yml          # CodeQL 安全扫描（每周一次）
    │   └── release.yml         # Tag 触发自动发布 + 源码打包
    ├── CODEOWNERS
    └── dependabot.yml
```

---

## CI/CD

本仓库使用 GitHub Actions 实现持续集成与发布：

### CI (`ci.yml`)

在 `pull_request`（目标 `main` / `develop`）和 `push`（`develop`）时触发：

| Job | 说明 |
|-----|------|
| **Encoding Check** | 验证所有源文件（`.cpp`, `.h`, `.hpp`, `.txt`, `.cmake`）为 UTF-8 编码 |
| **CMake Lint** | 使用 `cmakelang` 对所有 `CMakeLists.txt` 进行静态检查 |
| **Clang Format Check** | 检查 C++ 源文件格式一致性（仅告警，不阻断 CI） |

### CodeQL (`codeql.yml`)

在 `push`/`pull_request`（`main` / `develop`）及每周一 06:00 UTC 自动触发：

- 对 C++ 代码执行 `security-extended` 规则集的安全扫描

### Release (`release.yml`)

在推送 `v*` 格式的 Git tag 时触发：

1. 自动创建 GitHub Release，附带自上一个 tag 以来的变更日志
2. 打包源码为 `.tar.gz` 归档并上传至 Release

---

## 许可证

本项目采用 **Business Source License 1.1 (BSL 1.1)** 许可：

- **非生产用途**（开发、测试、评估、学术研究）：免费使用
- **生产用途**（制造检测系统部署、商业产品集成、营利性运营）：需获取商业许可
- **变更日期**：2030-01-01 后自动转为 **GPLv2**

商业许可请联系：rain3dmetrology@gmail.com

详见 [LICENSE](LICENSE) 文件。

---

## 公司

**苏州锐新视科技有限公司** (Suzhou Ruixin Vision Technology Co., Ltd.)
