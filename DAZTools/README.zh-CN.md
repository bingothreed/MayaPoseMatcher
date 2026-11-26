# DazToMaya —— DAZ ⇄ Maya 桥接（两个包，装完即用）

DAZ Studio 端负责**导出**，Maya 端负责**导入**，两边的版本必须配对（本包统一 `2024.2.3`）。

```
DazToMaya_DAZ端.zip    →  dsp_dzmayabridge.dll + 安装说明.txt       (DS6 插件，Qt6)
DazToMaya_Maya端.zip   →  DazToMaya.mod + DazToMaya\ + pymel wheel  (Maya 模块)
```

## 一、装 Maya 端（先装这个）

1. 解压 `DazToMaya_Maya端.zip`，把 **`DazToMaya.mod` 和 `DazToMaya` 文件夹一起**放到：

```
C:\Users\<你>\Documents\maya\2025\modules\
```

没有 `modules` 目录就新建。放 `Documents\maya\modules` 也可以，这两个位置 Maya 都会扫。

2. 装 PyMEL（`DAZ IMPORT` 按钮依赖它）：

```powershell
& "C:\Program Files\Autodesk\Maya2025\bin\mayapy.exe" -m pip install --user pymel-1.5.0-py2.py3-none-any.whl
```

3. 重启 Maya → Shelf 出现 **DazToMaya** 标签 → 里面有 **DAZ IMPORT**。

## 二、装 DAZ 端

解压 `DazToMaya_DAZ端.zip`，把 `dsp_dzmayabridge.dll` 复制到：

```
C:\Program Files\DAZ 3D\DAZStudio6\plugins\
```

（与 `dsp_dzhexagonbridge.dll` 等放一起，需要管理员权限）→ **完全退出 DS6 再启动** →
菜单出现 `File → Send To → Daz To Maya`。

> 这个 dll 是 **DS6 专用**（依赖 Qt6）。不要放进 DAZ Studio 4。

## 三、用起来

1. DAZ 里打开角色，衣服/头发先 **parent 到身体**。
2. `File → Send To → Daz To Maya`，选 **Skeletal Mesh**（带骨骼；只有网格选 Static）。
3. 要表情/形体滑杆就勾 **Export Morphs**；要细分勾 **Bake Subdivision**（级别别超过 2，会爆内存）。
4. Accept → 提示切到 Maya。
5. Maya 里点 Shelf 的 **DazToMaya** → **DAZ IMPORT** → **Auto-Import**。
6. 形变滑杆在 Outliner 的 `Morphs` 节点 → Attribute Editor 的 **Extra Attributes** 里。

## 四、常见问题

| 现象 | 处理 |
|---|---|
| Maya 里没 DazToMaya 标签 | 检查 `.mod` 和 `DazToMaya` 文件夹是否**同级**；Shelf 标签被隐藏时点 Shelf 齿轮 → Shelf Tabs |
| 点 DAZ IMPORT 报找不到 PyMEL | 在 `Documents\maya\2025\Maya.env` 加一行 `PYTHONPATH=%APPDATA%\Python\Python311\site-packages`，重启 Maya |
| DS6 没有 Send To 菜单 | 看 `%APPDATA%\DAZ 3D\Studio6\log.txt`；确认 dll 在 DS6 的 `plugins` 里，且没混入 DS4 版 |
| 导入后模型散架 | 衣服没 parent 到身体，或导出时选了 Static Mesh |
| 材质全白 | 两端版本不配对，或导出目录被移动过 |

卸载就删对应目录里的文件，没动任何系统设置。
