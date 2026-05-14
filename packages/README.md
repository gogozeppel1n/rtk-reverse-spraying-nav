# packages 目录说明

本目录按真实工程演化顺序保存各版本功能包。当前推荐主用版本是：

```text
10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```

01 和 02 的原始源码在用户上传的 RAR 中是展开目录；本次归档中保留了原始 RAR 到 `../original_archive/rtk-reverse-spraying-nav.original.rar`，并在 01/02 目录下补了说明文件。03 以后上传内容主要是 ZIP 包，本次已展开为可直接查看的目录。


## 编译注意

这里保存的是历史版本集合，不是一个可以整体放进 `catkin_ws/src` 编译的单一工作空间。由于多个版本内部可能都叫 `rtk_reverse_nav`，整体编译会出现 ROS package name 重复。

上车测试时，只复制一个目标版本包进入 `~/catkin_ws/src`。当前推荐复制：

```text
10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```
