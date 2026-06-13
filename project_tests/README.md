# 项目测试

这里放项目级测试工具，目标是验证一次真实采集 session 是否已经达到原型阶段的最低可用标准。

这些脚本和 `tests/` 不完全一样：

- `tests/` 偏向无硬件的单元测试和 CI 检查。
- `project_tests/` 偏向真实采集数据的离线验收。

## Session 质量检查

检查一段 dashboard 采集数据是否满足 `docs/prototype_plan.md` 里的通过标准：

- IMU timestamp 单调。
- camera frame group 有稳定 `group_id`。
- 四目 `skew_us` 不离谱。
- 可以重复采集短动作日志。

运行：

```bash
source .venv/bin/activate
python project_tests/session_quality/check_session_quality.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

当前样例：

```bash
python project_tests/session_quality/check_session_quality.py \
  --session-dir data/raw/session_20260612_174719
```

默认会输出 JSON 报告到：

```text
project_tests/reports/session_YYYYMMDD_HHMMSS_quality.json
```

如果要调整四目同步阈值：

```bash
python project_tests/session_quality/check_session_quality.py \
  --session-dir data/raw/session_YYYYMMDD_HHMMSS \
  --max-group-span-us 30000 \
  --max-abs-skew-us 20000
```

判断建议：

- `overall_ok: true`：这段数据可以进入 AprilTag / OpenVINS / wrist IMU 对齐的下一步。
- `overall_ok: false`：先看 `checks` 里失败的项目，再决定是重采还是调参数。

