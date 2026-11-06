---
description: Lynx S10 设备健康原语 — 电池 + SoC 温度遥测
---

# lynx_health — Lynx S10 device-health primitive

把 S10 的原生 drdds 遥测聚合进标准 `robonix/primitive/health` 数据面，供
Vitals 做阈值评估：

| S10 话题 | 类型 | 用途 |
|---|---|---|
| `/BATTERY_DATA` | `drdds/BatteryData` | 总电压、充放电、电池百分比、电芯温度 |
| `/cpu_status` | `drdds/CPUStatus` | 各 SoC 封装温度 |

## 接口

| 能力约定 ID | 模式 | 载荷 | 说明 |
|---|---|---|---|
| `robonix/primitive/health/state` | rpc | `health/GetHealthState` | 最新健康快照（gRPC） |
| `robonix/primitive/health/stream` | rpc_server_stream | `health/StreamHealthState` | 每 `interval_s` 推送一帧（gRPC 服务端流） |

## 字段映射

`HealthState`：

- `voltage`：电池总电压（V），`BatteryDataValue.voltage`（10 mV）换算。
- `charging`：`current < 0` 判定充电（符号启发式；需要硬信号时再看
  `/CHARGE_STATUS`）。
- `remaining_s`：放电时按 `remaining_capacity / current` 估算，否则 `-1`。
- `readings`：`battery`（temp_c=电芯均温、voltage、current_a、battery_percent）
  与每个 `cpu_<soc_id>`（temp_c=封装温度）。不可用字段一律 `-1`，不填 `0`。

## Config（部署清单 `config:`）

| key | default | 含义 |
|-----|---------|------|
| `battery_topic` | `/BATTERY_DATA` | 电池遥测话题 |
| `cpu_topic` | `/cpu_status` | CPU 温度话题 |
| `interval_s` | `1.0` | 健康流推送周期 |
| `sentinel_timeout_s` | `15.0` | 等待电池流的超时 |

## 注意

- `charging` 是电流符号启发式，不是 BMS 充放电状态机的权威结果。
- 故障事件（`/fault_event`）当前未聚合进 `HealthState`（该 IDL 没有专用故障
  字段）；如需，可在 `readings` 里追加一条 `faults` 读数或在 Vitals 侧扩展。
