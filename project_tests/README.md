# 项目测试

这里放真实采集数据的离线验收工具。它和 `tests/` 的区别是：

- `tests/`: 不依赖硬件的自动化单元测试。
- `project_tests/`: 对 `data/raw/session_*` 这类真实采集 session 做质量检查。

完整运行流程见 `../docs/system_workflow.md`。

## 当前检查

`session_quality/check_session_quality.py` 检查一段 dashboard session 是否满足原型最低可用标准：

- IMU timestamp 单调。
- camera frame group 有稳定 `group_id`。
- 四目 `skew_us` 不离谱。
- 短动作日志有足够时长和帧数。

默认报告输出到：

```text
project_tests/reports/session_YYYYMMDD_HHMMSS_quality.json
```

`project_tests/reports/` 默认不提交，用来保存本地数据报告。
