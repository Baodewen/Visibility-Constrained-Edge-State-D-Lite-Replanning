# Method: VCES-D* Lite

## 中文名称

**视野约束边状态 D* Lite 在线重规划方法**

## English name

**Visibility-Constrained Edge-State D* Lite Replanning**

## Abbreviation

**VCES-D* Lite**

## State representation

Each internal edge between adjacent cells has one of three states:

| State | Meaning | Cost |
|---|---|---|
| `UNKNOWN` | not scanned yet | finite, slightly higher than free |
| `FREE` | scanned and traversable | 1 |
| `BLOCKED` | scanned and blocked | infinity |

## Visibility model

The robot scans only the front near-field region:

```text
front 3×3 area relative to robot heading
+ line-of-sight occlusion check
+ no rear scan
```

## Execution policy

The candidate path may contain unknown edges, but the robot cannot move through an unknown edge.

```text
if no path:
    initialize / repair D* Lite
elif next edge is UNKNOWN:
    face next cell
    scan front 3×3
    update edge states
    repair path if costs changed
elif next edge is FREE:
    move one cell
elif next edge is BLOCKED:
    repair path
```
