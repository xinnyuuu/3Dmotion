# wam_token_writer

最终 motion stream 输出模块骨架。

## 目标

```text
head pose + wrist pose -> WAM motion token stream
```

输出格式见 `../../docs/data_schema.md` 和 `../../Context.md`。

## 当前状态

还未实现。后续负责把 `T_W_H`、`T_W_B`、速度、角速度和 tracking state 写成下游 World Action Model 可消费的 JSONL / binary stream。
