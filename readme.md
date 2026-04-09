# upperlimb_robot 控制函数使用说明

## 1. 控制方式概述

系统通过一个统一结构体 `ControlOutputCache` 来描述所有控制指令。

控制分为两类：

- 位置控制
- 扭矩控制

控制函数的本质，是对该结构体进行正确赋值。

---

## 2. 控制输出结构

```cpp
ControlOutputCache output;
```

关键字段如下：

| 字段 | 含义 |
|---|---|
| `valid` | 本轮控制是否生效 |
| `use_position[i]` | 第 i 个轴是否使用位置控制 |
| `use_torque[i]` | 第 i 个轴是否使用扭矩控制 |
| `target_position[i]` | 目标角度（deg） |
| `target_velocity[i]` | 目标速度（deg/s） |
| `target_acc[i]` | 加速度（deg/s²） |
| `target_dec[i]` | 减速度（deg/s²） |
| `target_torque[i]` | 目标扭矩（Nm） |

---

## 3. 位置控制用法

### 写法

```cpp
ControlOutputCache output{};

output.use_position[i] = true;

output.target_position[i] = 30.0;   // 角度：deg
output.target_velocity[i] = 20.0;   // 速度：deg/s
output.target_acc[i] = 100.0;       // 加速度：deg/s²
output.target_dec[i] = 100.0;

output.valid = true;
```

### 说明

- 只要 `use_position[i] = true`，该轴会进入位置控制模式
- 位置单位统一为 `deg`

---

## 4. 扭矩控制用法

### 写法

```cpp
ControlOutputCache output{};

output.use_torque[i] = true;

output.target_torque[i] = 5.0;   // 单位：Nm

output.valid = true;
```

### 说明

- 扭矩单位必须为 `Nm`
- 系统内部会自动转换为驱动单位

---

## 5. 同时控制多个轴

```cpp
ControlOutputCache output{};

output.use_position[10] = true;
output.target_position[10] = 20.0;

output.use_torque[11] = true;
output.target_torque[11] = 3.0;

output.valid = true;
```

说明：

- 每个轴可以独立选择控制方式
- 不同轴可以使用不同控制模式

---

## 6. 停止控制

```cpp
ControlOutputCache output{};
output.valid = false;
```

效果：

- 本轮不发送控制命令
- 系统会自动恢复安全状态

---

## 7. 轴编号说明

| 类型 | 范围 | 说明 |
|---|---|---|
| 控制索引 index | 0 ~ 19 | 用于数组访问 |
| 物理轴号 axis | 1 ~ 20 | 用于底层驱动 |

注意：

- 控制时使用的是 `index`
- 不需要手动处理 `axis`

---

## 8. 单位约定

所有控制输入统一使用物理单位：

| 类型 | 单位 |
|---|---|
| 角度 | deg |
| 角速度 | deg/s |
| 加速度 | deg/s² |
| 扭矩 | Nm |

---

## 9. 使用规则

1. 必须设置 `valid = true`，否则控制不会生效  
2. 位置控制使用 `use_position`  
3. 扭矩控制使用 `use_torque`  
4. 同一轴不建议同时开启两种控制  
5. 不需要手动切换轴模式，系统自动处理  

---

## 10. 最简模板

### 位置控制

```cpp
output.use_position[i] = true;
output.target_position[i] = xxx;
output.valid = true;
```

### 扭矩控制

```cpp
output.use_torque[i] = true;
output.target_torque[i] = xxx;
output.valid = true;
```

---

## 11. 总结

控制只依赖一个结构：

 `ControlOutputCache`

只需要：

- 选择控制类型（位置 / 扭矩）
- 填写目标值
- 设置 `valid = true`

其余流程由系统自动完成。
