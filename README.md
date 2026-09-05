# 🤖 工程训练搬运车 (Engineering Training Cart)

基于 STM32H723 的物流搬运机器人项目，用于工创搬运赛道比赛。

## 📋 项目概述

本项目是一个自主导航搬运机器人，用于完成物流搬运赛道任务。机器人需要按照规则完成货物搬运、路径规划等任务。

## 🏗️ 硬件平台

- **MCU**: STM32H723VGT6 (ARM Cortex-M7)
- **封装**: LQFP100
- **主频**: 550MHz
- **特性**: 
  - I-Cache 启用
  - MPU 配置
  - 调试接口 (SWD)

## 📁 项目结构

```
Engineering_Training_Cart/
├── MC02/                          # 主程序目录
│   ├── Core/                      # 核心代码
│   │   ├── Inc/                   # 头文件
│   │   └── Src/                   # 源文件
│   ├── Drivers/                   # 驱动库
│   │   ├── CMSIS/                 # ARM CMSIS 库
│   │   └── STM32H7xx_HAL_Driver/ # STM32 HAL 库
│   ├── cmake/                     # CMake 配置
│   ├── build/                     # 编译输出 (已忽略)
│   ├── CMakeLists.txt             # CMake 主配置
│   ├── STM32H723xG_flash.ld      # 链接脚本
│   └── startup_stm32h723xx.s      # 启动文件
├── 比赛规则文档/                    # 比赛规则 PDF
├── .gitignore                     # Git 忽略配置
└── README.md                      # 本文件
```

## 🚀 快速开始

### 环境要求

- **编译器**: ARM GCC (arm-none-eabi-gcc)
- **构建工具**: CMake 3.22+
- **IDE**: VSCode + STM32CubeMX (可选)
- **调试器**: ST-Link V2

### 编译步骤

```bash
# 1. 进入项目目录
cd MC02

# 2. 创建构建目录
mkdir build && cd build

# 3. 配置 CMake
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 4. 编译
make -j$(nproc)
```

### 烧录程序

```bash
# 使用 ST-Link 烧录
st-flash write build/Debug/MC02.bin 0x08000000

# 或使用 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program build/Debug/MC02.elf verify reset exit"
```

## 📐 硬件连接

### 调试接口

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA13 | SWDIO | 数据线 |
| PA14 | SWCLK | 时钟线 |
| GND   | 地线 | 公共地 |
| 3.3V  | 电源 | 3.3V 供电 |

### 时钟配置

- 外部高速晶振 (HSE): PH0/PH1
- 外部低速晶振 (LSE): PC15

## 📚 比赛规则

详细的规则文档请查看：
- [工创搬运赛道.pdf](工创搬运赛道.pdf)
- [物流搬运规则.pdf](物流搬运规则.pdf)
- [物流搬运评分规则.pdf](物流搬运评分规则.pdf)

## 🛠️ 开发工具

### STM32CubeMX

使用 STM32CubeMX 配置外设和引脚：
```bash
# 打开配置文件
stm32cubemx MC02/MC02.ioc
```

### VSCode 配置

项目包含 VSCode 配置文件，支持：
- C/C++ 智能提示
- Cortex-Debug 调试
- CMake 集成

## 📝 开发日志

- 2024-08-19: 项目初始化，完成基础框架搭建
- 2024-09-05: 添加比赛规则文档，完善项目配置

## 🤝 贡献指南

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 📄 许可证

本项目仅供学习和比赛使用。

## 👥 作者

- **Hh9** - *初始开发* - [Hh9-git](https://github.com/Hh9-git)

## 🙏 致谢

- STM32CubeMX 代码生成工具
- ARM CMSIS 库
- STM32 HAL 驱动库

---

📧 联系方式: 71461321@qq.com
