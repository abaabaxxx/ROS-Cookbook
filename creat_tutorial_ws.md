# 新建 tutorial_ws 工作空间

1.  **创建工作空间目录：**

    ```bash
    mkdir -p ~/tutorial_ws/src
    ```

2.  **进入工作空间目录并初始化 Catkin：**

    进入你刚刚创建的工作空间目录，然后初始化 Catkin 来构建ROS系统。

    说人话：让 ROS 的“管理者”catkin 能够“认识”你新创建的工作空间里的所有内容（你的 ROS 包和代码），并且知道如何将它们“组织”起来，准备好被 ROS 系统使用。

    ```bash
    cd ~/tutorial_ws
    catkin_make
    ```

3.  **设置环境变量 (Source setup.bash)：**

    为了让你的 ROS 系统能够找到并使用你新创建的工作空间中的 ROS 包，你需要“source” `devel/setup.bash` 文件。这会将当前 ROS 环境指向你的新工作空间。

    说人话：这个操作就是 “把我们新创建的工作空间的“使用说明书”（devel/setup.bash）递交给 ROS 系统这个“管理者”，并“告诉这个管理者：‘我们现在要用这个工作空间了，请你把它放到你的管理名单里，并且仔细看看里面的东西（包、可执行文件、库），知道它们在哪里，怎么用。’


    ```bash
    source devel/setup.bash
    ```

    **提示：** 将此命令添加到 `~/.bashrc`中，这样每次打开一个终端都会自动激活这个工作空间

    打开你的 `~/.bashrc` 文件：

    ```bash
    nano ~/.bashrc
    ```

    在文件末尾添加以下行：

    ```bash
    source ~/tutorial_ws/devel/setup.bash
    ```
