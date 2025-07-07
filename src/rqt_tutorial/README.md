# 教程：ROS 动态参数调节

## 步骤 1：创建 ROS 包

```bash
cd ~/tutotial_ws/src  
catkin_create_pkg rqt_tutotial rospy dynamic_reconfigure
```

> 注意：如果创建包时忘记添加 `dynamic_reconfigure` 依赖，可以手动补充。我们只需要修改两个文件：`package.xml` 和 `CMakeLists.txt`。

**1. 修改 `package.xml`**

**非人话**：声明 `dynamic_reconfigure` 为构建和运行时依赖。
**说人话**：告诉 ROS，在编译和运行这个包时，需要用到动态参数功能。

```xml
<build_depend>dynamic_reconfigure</build_depend>
<exec_depend>dynamic_reconfigure</exec_depend>
```

**2. 修改 `CMakeLists.txt`**

(1) 在 `find_package` 中添加 `dynamic_reconfigure`

**非人话**：这样 CMake 才能识别 `generate_dynamic_reconfigure_options` 命令。
**说人话**：告诉 CMake，在构建这个包时，需要这个依赖包，才能支持动态参数功能。

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure  # <----- 添加这一行
)
```

(2) 在 `catkin_package` 中添加 `CATKIN_DEPENDS`

**非人话**：这能确保当其他包依赖你的包时，相关依赖能正确传递。
**说人话**：当别人依赖你这个包时，如果没有 `dynamic_reconfigure`，功能就会出错。

```cmake
catkin_package(
  CATKIN_DEPENDS rospy dynamic_reconfigure  # <----- 添加这一行
)
```
(3)  `catkin_package` 中添加生成规则 

**非人话**：让 catkin_make 在编译时根据你的 `.cfg` 脚本，自动生成可以 import 的 `rqt_tutorialConfig.py` 文件。
**说人话**：诉施工队（catkin_make）去读取这张参数表(`.cfg`)，并根据它来生成实际的`python`代码
```cmake
# 添加动态参数的生成规则
generate_dynamic_reconfigure_options(
  cfg/rqt_tutorial.cfg   # 注意：这里的路径和文件名必须与实际创建的 .cfg 文件保持一致
)
```
---
## 步骤 2：定义参数（`.cfg` 文件）

本次示例中，我们将定义两组参数：**速度组** 和 **位置组**，并通过 `.cfg` 文件进行配置。

按照 ROS 的规范，需要在当下功能包目录下创建一个名为 `cfg` 的文件夹（这是标准做法），并在其中创建参数配置文件 `rqt_tutorial.cfg`。

详细代码请见仓库中的 `cfg/rqt_tutorial.cfg` 文件。

**注意：请为 `.cfg` 文件添加执行权限！**

```bash
chmod +x cfg/rqt_tutorial.cfg
```

---
## 步骤 3：编写ROS节点 (`rqt_tutorial_node.py`)

**目标说明：**

1. **创建一个 Python 脚本**，作为 ROS 节点使用。
2. **导入 `.cfg` 文件自动生成的配置类**（`rqt_tutorialConfig`）。

   * 导入该配置类后，节点即可访问 `.cfg` 文件中定义的所有动态参数。
3. **启动动态调参服务**（`dynamic_reconfigure.server.Server`）。

   * 启动该服务后，节点将具备与 `rqt_reconfigure` 图形界面实时交互的能力。
   * 当参数在界面中被修改时，该服务会自动触发绑定的回调函数。
4. **编写回调函数**，用于响应参数变化，获取并处理最新的参数值。
5. **效果展示**：在节点的主循环中打印当前参数值，便于观察调参结果的实时变化。

**详细代码请见rqt_tutorial_node.py**
---

## 步骤 4：测试

使用 `launch` 文件启动节点，验证动态参数是否能在运行过程中被正确调节。


```bash
roslaunch rqt_tutorial rqt_tutorial.launch
```



