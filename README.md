# OpenCV Calibration 使用教程

基于 OpenCV + yaml-cpp 的标定工具，支持三种输入源：

- `HIK`：海康工业相机
- `UVC`：USB 相机（`/dev/video*`）
- `IMG`：离线图片目录

项目提供 4 个可执行程序：

- `calibrateCamera`：相机内参标定
- `calibrateHandEye`：手眼标定
- `calculateError`：查看实时重投影误差
- `validateHandEye`：手眼标定结果验证

---

## 1. 环境依赖

Ubuntu/Debian：

```bash
sudo apt update
sudo apt install -y cmake g++ libopencv-dev libyaml-cpp-dev libfmt-dev libeigen3-dev
# 写入海康相机 udev 规则
sudo tee /etc/udev/rules.d/80-drivers-SDK-2bdf.rules >/dev/null <<'EOF'
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF
# 重新加载规则
sudo udevadm control --reload-rules
# 触发当前已连接设备的规则应用
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=2bdf
```

---

## 2. 编译

在项目根目录执行：

```bash
cmake -S . -B build
cmake --build build -j
```

编译完成后可执行文件位于 `build/` 目录。

---

## 3. 先改配置（必须）

主配置文件：`config/calibration.yaml`

重点字段：

1. 输入源选择

```yaml
device: HIK   # 可选: HIK / UVC / IMG
```

2. 标定板参数（必须和实际标定板一致）

```yaml
pattern: chessboard   # chessboard / circles / acircles
pattern_rows: 8
pattern_cols: 8
square_size: 35       # 单位 mm
```

3. 对应输入源参数

- `HIK`：曝光、增益、帧率等
- `UVC`：`video_path`、分辨率、帧率
- `IMG`：`images_path`（离线图片目录）

4. 数据保存路径

```yaml
camera_calib_save_path: ./camera_calib_images
handeye_calib_save_path: ./handeye_calib_data
```

5. 手眼标定串口参数（仅手眼流程需要）

```yaml
Serial:
	port_name: /dev/rm_usb0
	baud_rate: 115200
```

---

## 4. 运行前说明（WebViewer）

程序界面通过内置 Web 服务显示，默认端口 `8080`。

1. 启动程序后，在浏览器打开：`http://localhost:8080`
2. 点击页面使其获得焦点
3. 再使用键盘快捷键（`s/a/c/r/ESC`）

---

## 5. 相机标定（calibrateCamera）

### 启动

```bash
./build/calibrateCamera
```

指定配置文件：

```bash
./build/calibrateCamera --config-path=config/calibration.yaml
```

### 操作键

- `s`：手动采集当前帧
- `a`：切换自动采集
- `c`：开始计算内参
- `ESC`：退出

### 输出

- 标定图片：`camera_calib_save_path/image_*.jpg`
- 内参文件：`camera_calibration.yaml`（项目根目录）

---

## 6. 手眼标定（calibrateHandEye）

> 建议先完成相机内参标定，并将内参/畸变参数写入 `config/calibration.yaml`。

### 6.1 在线采集并标定

```bash
./build/calibrateHandEye
```

操作键：

- `s`：采集一组手眼数据（图像 + 云台姿态）
- `c`：开始计算手眼参数
- `ESC`：退出

输出：

- 采集数据：`handeye_calib_save_path/image_*.jpg`、`handeye_calib_save_path/pose_*.yaml`
- 标定结果：`handeye_calibration.yaml`（项目根目录）

### 6.2 从历史数据直接重算（离线）

```bash
./build/calibrateHandEye -l -d ./handeye_calib_data
```

说明：目录内需成对存在 `image_xxx.jpg` 与 `pose_xxx.yaml`。

---

## 7. 重投影误差查看（calculateError）

```bash
./build/calculateError
```

用途：实时显示 `RMSE(px)`，用于评估当前内参在实际图像中的重投影效果。

---

## 8. 手眼结果验证（validateHandEye）

```bash
./build/validateHandEye
```

或指定手眼结果文件：

```bash
./build/validateHandEye --handeye-path=handeye_calibration.yaml
```

验证方式：固定标定板、转动云台，观察“世界坐标系下标定板位置一致性”。

- `r`：重置统计
- `ESC`：退出

重点看两项：

- `Position StdDev`：越小越稳定
- `Reprojection Error`：越小越好

---

## 9. 推荐完整流程

1. 修改 `config/calibration.yaml`（输入源、标定板参数、串口）
2. 运行 `calibrateCamera`，得到 `camera_calibration.yaml`
3. 将内参与畸变参数更新到 `config/calibration.yaml`
4. 运行 `calibrateHandEye` 采集并计算，得到 `handeye_calibration.yaml`
5. 运行 `validateHandEye` 做结果稳定性验证

---

## 10. 常见问题

- 浏览器无画面：确认程序已启动并访问 `http://localhost:8080`
- 按键无效：先点击网页让页面获得焦点
- 一直提示图像为空：检查 `device` 配置及设备路径/图片路径
- 手眼标定效果差：确保采集姿态丰富，且采集时标定板在世界坐标系中保持固定

---

## 11. 致谢
本项目基于 仲恺农业学院开源标定程序 [OpenCV Calibration — 相机与手眼标定工具集](https://gitee.com/slime0rimiru0/open-cv_-calibration) 修改而来
