# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

boneDynamicsNode 是一个 Autodesk Maya 的 C++ 插件,对关节链(bone–end 关节段)施加动力学模拟(重力、弹性、阻尼、角度限制、碰撞、湍流风),驱动骨骼产生物理动画。分三层交付:

- **C++ 插件**(`src/`):核心模拟节点和碰撞体/可视化节点
- **Python 辅助包**(`scripts/bdn/`):供美术/脚本使用的装配 API
- **预编译 `.mll`**(`plug-ins/<年份>/`):为 Maya 2022–2027 各版本提供

## 构建(无测试框架)

运行 `tests/maya_regression.py` 和 `tests/mesh_collider_regression.py`，参数为重编译 `.mll` 的绝对路径，使用 Maya 2022 的 mayapy。根部降级回归见 `tests/mesh_root_fallback_regression.py`，真实 ABC 验收见 `tests/mesh_root_fallback_abc.py`。性能冒烟检查见 `tests/mesh_collider_benchmark.py`。视口仍需在交互式 Maya 中验证。

### Windows 全版本构建

1. 编辑 `build_win.bat` 顶部的配置:`GENERATOR`(VS 生成器)和 `DEVKIT_ROOT`(Maya Devkits 根目录,其中需存在匹配 `Autodesk_Maya_<年份>*_DEVKIT_Windows` 模式的目录)
2. 运行 `build_win.bat`,它会为 2022–2027 各版本逐个执行 cmake 配置 + 构建
3. 产物在 `build/win64_<年份>/`,将其中的 `boneDynamicsNode.mll` 复制到 `plug-ins/<年份>/` 以更新预编译版本

### 单版本手动构建

```bat
set "DEVKIT_LOCATION=D:\MayaDev\Autodesk_Maya_2027*_DEVKIT_Windows\devkitBase"
cmake -S src -B build/win64_2027 -G "Visual Studio 18 2026" -A x64 -T v143 -DCXX_STANDARD_VER=20
cmake --build build/win64_2027
```

- `src/CMakeLists.txt` 通过 `$ENV{DEVKIT_LOCATION}` 找到 devkit 的 `pluginEntry.cmake`,因此手动构建必须预先设置该环境变量
- 平台差异:Windows 是开发和测试平台;Linux 仅在 WSL 下验证过编译;macOS 未测试

## 架构

### C++ 插件(`src/`)

`pluginMain.cpp` 是唯一入口，注册/注销 **6 个节点和 1 个自定义数据类型**。locator 碰撞体/可视化器还注册视口绘制器：

| 节点 | 类型 | 作用 |
|---|---|---|
| `boneDynamicsNode` | `MPxNode` | 核心模拟节点,`compute()` 中执行动力学积分与约束求解 |
| `sphereColliderNode` / `capsuleColliderNode` / `infinitePlaneColliderNode` | `MPxLocatorNode` | 三种碰撞体,`colliderNode/` 目录 |
| `boneDynamicsVisualizer` | `MPxLocatorNode` | 显示模拟出的末端姿态,`visualizerNode/` 目录 |
| `meshColliderNode` | `MPxNode` | 共享网格快照、子帧采样、原生查询缓存，输出 `MeshColliderData` |

- `boneDynamicsNode.{h,cpp}`:所有动力学/变换/碰撞参数都是该节点的静态 `MObject` 属性(`s_*`,见头文件中的分组注释);模拟状态(`m_position`、`m_velocity`、湍流 RNG 状态)保存在节点实例中
- `boneDynamicsUtils.{h,cpp}`:纯数学层,`BoneInput` → `PoseData` 的构建;被主节点和可视化节点共享,不含 Maya 依赖之外的逻辑
- `mathUtils.{h,cpp}`:底层数学工具
- `nodeIds.h`:所有节点 ID 的唯一出处
- `colliderNode/meshCollision.*`:MMeshIntersector 缓存、原生 swept-AABB broad phase、变半径胶囊/三角面 CCD 和接触约束。设计及限制见 `docs/mesh_colliders.md`。

### Python 辅助包(`scripts/bdn/`)

`bdn` 包通过 Maya 的 `cmds` 实现程序化装配,公共 API 从 `__init__.py` 导出:

- `core.py`:`create_dynamics_node()` — 核心装配函数(创建节点、连接关节矩阵、支持缩放/目标姿势/偏移节点/碰撞体/可视化器,见函数 docstring)
- `collider.py`:创建球体、胶囊、平面和共享 Mesh 碰撞体，以及 `connect_colliders` / `disconnect_colliders`
- `visualize.py`:创建可视化器
- `utils.py`:公共工具,`load_plugin()`(自动加载 .mll)、`undo_chunk`、`with_traceback` 装饰器
- `version.py`:`__version__`

`sample_scripts/` 展示了标准用法:选中关节链(根→尖端),对每段 `bone–end` 调用 `bdn.create_dynamics_node(bone, end)`。

### 其他

- `scripts/AEboneDynamicsNodeTemplate.mel`:节点在属性编辑器中的模板
- `plug-ins/<年份>/`:预编译 mll;发布新版本时需更新全部年份并同步 README 中的下载链接

## 关键约定

- **节点 ID 是内部使用 ID**(`0x7b001`–`0x7b009`,见 `nodeIds.h`),非 Autodesk 分配;新增节点必须在 `nodeIds.h` 分配未使用的 ID,README 顶部有警告
- 旋转顺序固定为 `kXYZ`(`boneDynamicsUtils.h` 中的 `kRotationOrder`)
- 插件版本号在 `pluginMain.cpp` 的 `MFnPlugin` 构造函数中;Python 包版本在 `scripts/bdn/version.py`(两者相互独立)
- 新增属性时:在 `boneDynamicsNode.h` 声明静态 `MObject` → 在 `.cpp` 的 `initialize()` 中 `addAttribute` → 在 `compute()` 中读取/响应
- C++ 需要兼容 Maya 2022(MSVC v142、C++17)到 2027(v143、C++20),使用新 API 时需留意版本差异
