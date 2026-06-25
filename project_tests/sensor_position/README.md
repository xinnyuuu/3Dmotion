# Sensor Position Tests

这个目录用于记录手部摄像头和 IMU 候选安装位的真实测试结果。长期需求和评分规则见：

```text
../../docs/hand_sensor_position_requirements.md
```

## 文件说明

- `candidate_matrix.csv`: 预置的候选位置矩阵，测试前先复制一份到 `project_tests/reports/` 或本地记录目录再填写。
- `session_log_template.csv`: 每段实际采集 session 的记录表模板。
- `scorecard_template.md`: 汇总截图、视频、评分和结论的 Markdown 模板。

## 推荐流程

1. 按 `candidate_matrix.csv` 逐个搭建候选安装位。
2. 每个候选位置录 3 段 `20-30 s` session。
3. 运行：

```bash
python scripts/validate_session.py --session-dir data/raw/session_YYYYMMDD_HHMMSS
```

4. 把 session 路径、截图、视频片段和评分填到模板里。
5. 按 `推荐 / 备选 / 仅手部观察位 / 不推荐` 输出给机械团队。

## 评分提醒

如果候选位置要被写成 VIO 位，必须同时满足：

- 摄像头画面里有足够稳定环境纹理。
- IMU 与摄像头属于同一刚体。
- 相机-IMU 时间戳可以映射到同一时间轴。
- 内参、外参和时间偏移都能标定。

否则该位置只能写成手部观察位或动作辅助位。
