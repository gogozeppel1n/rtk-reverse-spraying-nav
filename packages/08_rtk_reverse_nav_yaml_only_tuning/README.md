# 08_rtk_reverse_nav_yaml_only_tuning

## 定位

该目录是占位版本。原思路是把可调参数集中到 YAML，现场只改 `config/nav_params.yaml`，避免每次都改 launch 或 Python 主控代码。

## 为什么不单独放源码

这个阶段更多是“调参组织方式”的整理，不是一个新的稳定算法分支。它的思想已经被 09 和 10 吸收：

- `reverse_nav.launch` 只加载 YAML；
- 现场调参只改 `config/nav_params.yaml`；
- 不再由 launch 参数覆盖关键控制参数。

当前应直接看 10 号版本。
