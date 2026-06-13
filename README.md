# Embedded System Project: Real-Time Fan Vibration Anomaly Detection

## 📋 項目概述

本項目是一個基於 **Raspberry Pi 5 + PREEMPT_RT Linux 內核** 的實時風扇振動異常檢測系統。使用 **MPU6050 三軸加速度計** 監測設備（風扇）的振動特性，通過實時信號分析和機器學習算法檢測異常狀況，實現預測性維護。

**核心目標：**
- ✅ 實時採集設備振動數據
- ✅ 計算振動特徵（RMS、諧波比、頻率等）
- ✅ 異常檢測與告警
- ✅ 低延遲實時性（PREEMPT_RT）
- ✅ 實時性能監測與分析

---

## 🔧 硬件要求

| 組件 | 規格 | 備註 |
|------|------|------|
| 主板 | Raspberry Pi 5 | 或相容產品 |
| 加速度計 | MPU6050（6軸 IMU） | I2C 地址: 0x68 |
| 通訊界面 | I2C-1 | /dev/i2c-1 |
| 按鈕 | GPIO 17 | 用於手動觸發異常 |
| 操作系統 | Linux PREEMPT_RT | 用於實時性保證 |

**連接圖：**
```
Raspberry Pi 5
  ├─ I2C-1 (SDA/SCL) ──→ MPU6050
  └─ GPIO 17 ──→ Button (wired to GND)
```

---

## 📁 文件結構

```
ESProject/
├── main.c                          # 主程序 - 實時數據採集與異常檢測
├── mpu6050_driver.c               # MPU6050 I2C 驅動程序
├── mpu6050_driver                 # 已編譯的驅動模塊
├── mpu6050-overlay.dts            # 設備樹覆蓋文件（設備配置）
├── mpu6050.dtbo                   # 編譯後的設備樹二進製對象
├── fan_vibration_rt               # 已編譯的主程序執行文件
│
├── plot_fan_log.py                # 繪製振動分析圖表
├── plot_score_from_log.py         # 繪製異常分數曲線
├── plot_score_live.py             # 實時動態繪圖
│
├── fan_log.txt                    # 實時採集的振動數據日誌
├── fan_log_charts/                # 生成的數據分析圖表
│   ├── score_line.png             # 異常分數變化曲線
│   ├── frequency_line.png         # 主頻率變化
│   ├── rms_vibration_line.png     # RMS 振幅變化
│   ├── harmonic_ratio_line.png    # 諧波比變化
│   ├── harmonic_amplitude_line.png # 諧波幅度變化
│   ├── rt_latency_line.png        # 實時延遲
│   └── rt_event_counts.png        # 實時事件計數
│
├── Embedded System proposal.pptx  # 項目提案文檔
└── project_result_presentation.pptx # 最終成果演示
```

---

## 🏗️ 軟件架構

### 核心模塊

#### 1. **Main Program (main.c)**
- **功能：** 
  - 初始化 I2C 通訊和 GPIO
  - 創建實時線程進行數據採集
  - 計算振動特徵指標
  - 異常檢測與評分
  
- **實時性特性：**
  - 使用 `pthread` 和 `sched.h` 實現 PREEMPT_RT 優先級調度
  - 採用固定 1kHz 採樣率採集加速度數據
  - 實時延遲監測

#### 2. **MPU6050 驅動 (mpu6050_driver.c)**
- I2C 總線通訊
- 傳感器初始化和喚醒
- 加速度數據的 Burst Read（連續讀取）
- WHO_AM_I 驗證以確保硬件連接正常

#### 3. **設備樹覆蓋 (mpu6050-overlay.dts)**
- 定義 MPU6050 設備節點
- 配置 I2C 參數
- 通過 Device Tree 向 Linux 內核註冊硬件

---

## 📊 核心功能

### 1. **實時數據採集**
```
採樣率: 1kHz
數據類型: 三軸加速度 (X, Y, Z)
數據格式: int16_t (±32768)
```

### 2. **振動特徵提取**

| 特徵 | 計算方式 | 用途 |
|------|---------|------|
| **RMS (均方根)** | √(Σx²/N) | 振動幅度指標 |
| **頻率分析** | FFT 變換 | 識別主頻率和諧波 |
| **諧波比** | 諧波幅度/基頻幅度 | 異常特徵 |
| **異常分數** | 加權組合 | 綜合異常程度 |

### 3. **異常檢測算法**
- **基線建立期：** 前 512 幀用於建立正常設備基線
- **實時判斷：** 將當前指標與基線比對
- **多維評分：** 結合 RMS、諧波比、頻率偏差等因素
- **閾值告警：** 超過預設閾值時觸發異常警報

### 4. **實時性監測**
- 記錄每幀採集延遲（ms）
- 統計實時事件發生次數
- 監控系統實時性能表現

---

## 🚀 構建與運行

### 編譯主程序
```bash
cd ESProject
gcc -O2 -Wall -Wextra -pthread main.c -lm -o fan_vibration_rt
```

**編譯選項說明：**
- `-O2` ：中等優化級別
- `-Wall -Wextra` ：啟用所有警告
- `-pthread` ：鏈接 POSIX 線程庫
- `-lm` ：鏈接數學庫（用於 FFT 計算）

### 運行程序
```bash
# 需要 root 權限存取 I2C 和 GPIO
sudo ./fan_vibration_rt > fan_log.txt

# 或後臺運行
sudo nohup ./fan_vibration_rt > fan_log.txt 2>&1 &
```

### 加載設備樹
```bash
# 編譯設備樹源文件
dtc -@ -I dts -O dtb -o mpu6050.dtbo mpu6050-overlay.dts

# 加載設備樹覆蓋
sudo dtoverlay mpu6050.dtbo

# 驗證設備
i2cdetect -y 1  # 應顯示 0x68 處有設備
```

---

## 📈 數據分析和可視化

### 生成分析圖表
```bash
python3 plot_fan_log.py fan_log.txt --config-frame 512 --out-dir fan_log_charts
```

**生成的圖表：**
1. **score_line.png** - 異常分數趨勢
2. **frequency_line.png** - 主振動頻率變化
3. **rms_vibration_line.png** - 振幅 RMS 曲線
4. **harmonic_ratio_line.png** - 諧波比分析
5. **harmonic_amplitude_line.png** - 諧波幅度
6. **rt_latency_line.png** - 實時延遲分佈
7. **rt_event_counts.png** - 實時事件計數

### 實時動態繪圖
```bash
python3 plot_score_live.py fan_log.txt
```
實時監視異常分數變化

---

## ⚙️ 配置參數

主程序中的關鍵配置（見 `main.c`）：

```c
/* 採樣配置 */
#define SAMPLING_RATE    1000        // 1kHz
#define BASELINE_FRAMES  512         // 前 512 幀作為基線

/* 異常檢測閾值 */
#define ANOMALY_THRESHOLD  2.5       // 異常分數閾值
#define RMS_THRESHOLD      1.5       // RMS 幅度閾值

/* 實時性配置 */
#define REALTIME_PRIORITY  50        // 實時優先級
#define LATENCY_WARN_MS    10        // 延遲警告值
```

**調整建議：**
- 根據實際設備收集數據後，使用 plot 腳本分析基線分佈
- 調整閾值以平衡檢測靈敏度和誤報率
- 在生產環境中進行長期測試驗證

---

## 📝 日誌格式

`fan_log.txt` 日誌示例：
```
[Frame 1] RMS: 12.45 Hz: 150.2 Score: 0.85 Latency: 2.3ms Events: 0
[Frame 2] RMS: 12.47 Hz: 150.1 Score: 0.87 Latency: 2.1ms Events: 0
[Frame 3] RMS: 12.50 Hz: 150.3 Score: 0.92 Latency: 2.4ms Events: 0
...
[Frame 512] (Baseline 完成，進入異常檢測模式)
[Frame 513] RMS: 12.52 Hz: 150.2 Score: 0.89 Latency: 2.2ms Events: 0
```

---

## 🔍 常見問題

### Q: 編譯出現 "cannot find -lm"
**A:** 安裝 math 庫開發包
```bash
sudo apt install libm-dev  # 或其他發行版的對應包
```

### Q: 運行時提示 "Permission denied"
**A:** 需要 root 權限或適當的用戶組權限
```bash
sudo ./fan_vibration_rt
```

### Q: I2C 設備未檢測到
**A:** 檢查設備樹和連接
```bash
i2cdetect -y 1
dtoverlay mpu6050.dtbo  # 確保設備樹已加載
```

### Q: 實時延遲過高
**A:** 
- 確認使用 PREEMPT_RT 內核
- 檢查其他系統進程負載
- 調整線程優先級

---

## 📚 相關技術

- **實時操作系統** - PREEMPT_RT Linux
- **嵌入式通訊** - I2C 協議
- **信號處理** - FFT、RMS、頻域分析
- **異常檢測** - 多特徵融合評分
- **實時調度** - POSIX 線程、優先級繼承

---

## 📄 相關文檔

- 項目提案：[Embedded System proposal.pptx](./Embedded%20System%20proposal.pptx)
- 成果演示：[project_result_presentation.pptx](./project_result_presentation.pptx)

---

## 👤 作者信息

**開發者：** Medrano  
**平台：** Raspberry Pi 5 + Linux PREEMPT_RT  
**最後更新：** 2026-06-14

---

## 📞 支援與反饋

如有問題或建議，歡迎提交 Issue 或 Pull Request。

---

**⭐ 如果此項目對您有幫助，請給予 Star 支持！**
