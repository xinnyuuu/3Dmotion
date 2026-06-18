# 技术文档索引

这个目录只保留对系统长期有用的技术文档。日常操作从 `system_workflow.md` 开始；其它文档按主题补充细节。

## 推荐阅读顺序

1. `system_workflow.md`
   - 从环境配置、dashboard 采集、session 检查到 AprilTag / OpenVINS 离线处理的完整操作入口。

2. `architecture.md`
   - 系统模块、数据流、OpenVINS 和 MOLA 在项目里的角色。

3. `coordinate_frames.md`
   - `W`、`H`、`B`、`I_H`、`I_B` 等坐标系定义，以及 `T_A_B` 的方向约定。

4. `calibration.md`
   - camera intrinsics、camera-to-head、IMU-to-rigid-body、AprilTag bracelet geometry、time offset 的标定清单。

5. `timestamp_sync.md`
   - host timestamp、device timestamp、四目 skew、camera-IMU offset 的处理策略。

6. `data_schema.md`
   - raw session、intermediate JSONL、OpenVINS bridge、最终 WAM motion stream 的数据格式。

7. `prototype_plan.md`
   - P0-P5 阶段目标和验收标准。未实现部分保留为骨架，不重复日常命令。

## 文档维护规则

- 操作流程只写在 `system_workflow.md`，其它文档只保留必要引用。
- 模块 README 只写模块职责、输入输出和当前状态，不复制完整命令流程。
- 临时讨论、方案对比、一次性 debug 记录不进入长期文档。
- 真实数据报告放在 `project_tests/reports/`，默认不提交。
