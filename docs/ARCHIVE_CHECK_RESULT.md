# ARCHIVE_CHECK_RESULT：本次归档检查结果

## 1. 原始上传内容

原始文件：

```text
rtk-reverse-spraying-nav.rar
```

RAR 内可识别的目录包括：

```text
packages/01_rtk_reverse_nav_save_dir_baseline/
packages/02_rtk_reverse_nav_pause_relay_initial_problematic/
packages/03_rtk_reverse_nav_stable_pause_relay/
packages/04_rtk_reverse_nav_shadow_pause_relay/
packages/05_rtk_reverse_nav_shadow_payload_optimized/
packages/06_rtk_reverse_nav_random_payload_shadow_final/
packages/07_rtk_reverse_nav_pause_yaw_hold_slew/
packages/08_rtk_reverse_nav_yaml_only_tuning 不放了，单纯pid调参/
packages/09_rtk_reverse_nav_spray_heading_stable/
packages/10_rtk_reverse_nav_spray_cte_keep_final/
docs/
support_packages/rtk_trajectory_exporter/
```

其中 03、04、05、06、07、09、10 在原始 RAR 内是 ZIP 条目，本次已展开为可直接查看的目录。01、02 在原始 RAR 内是压缩文本条目，本次保留原始 RAR 到：

```text
original_archive/rtk-reverse-spraying-nav.original.rar
```

后续可在本地用 WinRAR/7-Zip/unrar 展开后补齐 01、02 源码目录。

## 2. 已补齐文档

本次新增或补齐：

```text
README.md
packages/README.md
packages/01_rtk_reverse_nav_save_dir_baseline/README.md
packages/02_rtk_reverse_nav_pause_relay_initial_problematic/README.md
packages/08_rtk_reverse_nav_yaml_only_tuning/README.md
docs/VERSION_HISTORY.md
docs/PARAMETER_TUNING_GUIDE.md
docs/FIELD_TEST_NOTES.md
docs/ARCHIVE_CHECK_RESULT.md
```

## 3. Python 静态检查

对已展开的 03、04、05、06、07、09、10 版本 Python 文件执行了 `py_compile` 语法检查，结果均通过。该检查只代表 Python 语法层面无错误，不代表 ROS 运行时依赖、话题、串口、底盘安全全部通过。

## 4. 当前推荐使用

当前推荐主线仍然是：

```text
packages/10_rtk_reverse_nav_spray_cte_keep_final/rtk_reverse_nav
```

理由：它是喷洒直线 cte_keep 优化版，适合当前“固定作业航向 + 横差小航向偏置 + 停车前保留部分横差修正”的真实工况。
