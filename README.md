# my_r2d2

当前范围：`Ubuntu 24.04 + ROS 2 Jazzy` 上的 `nav2_costmap_2d` 真实 `topic/service/parameter` 闭环。

配置检查：

```bash
scripts/check_env.sh
```

工作区构建：

```bash
scripts/build_nav2_ws.sh
```

库测试：

```bash
cargo test
```

接口提取和生成回归：

```bash
cargo test --test interface_file_extractor --test mutation --test payload_generator
```

最小 dry run：

```bash
ROS_DOMAIN_ID=193 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 0 --seed-dir tests/fixtures/nav2_seeds
```

短轮运行：

```bash
ROS_DOMAIN_ID=193 cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 3 --seed-dir tests/fixtures/nav2_seeds --lcov-dir nav2_ws/results
```

全量运行：

```bash
ROS_DOMAIN_ID=200 cargo run --example nav2_costmap_e2e -- --rounds 1000 --oracle-mode jazzy-reproduction --seed-dir tests/fixtures/nav2_seeds --lcov-dir nav2_ws/results
```

保守口径运行：

```bash
ROS_DOMAIN_ID=201 cargo run --example nav2_costmap_e2e -- --rounds 1000 --oracle-mode paper-supported --seed-dir tests/fixtures/nav2_seeds --lcov-dir nav2_ws/results
```
