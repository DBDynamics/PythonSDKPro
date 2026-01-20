# DBDynamicsPro 3DOF Control

本项目包含用于 3DOF 控制的 Python 脚本。

## 环境要求

- Python 3.8 或更高版本

## 安装依赖

请在项目根目录下运行以下命令安装必要的 Python 库：

```bash
pip install -r requirements.txt
```

`requirements.txt` 包含了以下关键库：
- `scipy`: 用于科学计算和插值 (PchipInterpolator)
- `pyusb`: 用于 USB 通信
- `pyserial`: 用于串口通信

## USB 驱动配置 (Windows)

本项目使用 `pyusb` 进行 USB 通信。在 Windows 上，你需要为 USB 设备安装 libusb 驱动。

1.  找到项目目录下的 `driver` 文件夹。
2.  运行 `zadig-2.9.exe` (如果目录下没有，请从 [Zadig 官网](https://zadig.akeo.ie/) 下载)。
3.  在 Zadig 中：
    - 点击 **Options** -> **List All Devices**。
    - 在下拉列表中选择你的 USB 设备。
    - 确保目标驱动 (右侧) 选择为 **libusb-win32** 或 **WinUSB**。
    - 点击 **Replace Driver** 或 **Install Driver** 按钮。

完成驱动安装后，`pyusb` 才能正常识别和与设备通信。

## 运行

运行主程序：

```bash
python DBDynamicsPro.py
```
