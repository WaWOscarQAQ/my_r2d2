# R2D2 Rust core

当前只实现 `interface_extractor`，用于为后续 payload 生成提供统一的接口描述。

## 当前边界

- 论文明确描述：R2D2 会提取 ROS interfaces，并依据 interface specification 处理结构化输入。
- 工程占位实现：本模块暂不连接 ROS 2 graph，也不解析 `.msg`、`.srv` 或 `.action` 文件。
- 测试替身：`MockExtractor` 只存在于 `tests/`，不会进入正式库的公开 API。

## 运行测试

```text
cargo test
```

后续接入真实 ROS 2 时，只需新增一个实现 `Extractor` trait 的类型。
