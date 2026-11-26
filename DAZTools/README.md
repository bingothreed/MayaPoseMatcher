# DazToMaya — DAZ ⇄ Maya Bridge (two packages, ready to install)

The DAZ Studio side **exports**; the Maya side **imports**. Both sides must use the same
bridge version — both packages here are `2024.2.3`.

```
DazToMaya_DAZ端.zip    →  dsp_dzmayabridge.dll + 安装说明.txt       (DS6 plugin, Qt6)
DazToMaya_Maya端.zip   →  DazToMaya.mod + DazToMaya\ + pymel wheel  (Maya module)
```

> 中文说明见 [README.zh-CN.md](README.zh-CN.md)

## 1. Install the Maya side first

1. Unzip `DazToMaya_Maya端.zip` and put **both** the `DazToMaya.mod` file and the
   `DazToMaya` folder into:

```
C:\Users\<you>\Documents\maya\2025\modules\
```

Create the `modules` folder if it does not exist. `Documents\maya\modules` also works —
Maya scans both locations.

2. Install PyMEL (the `DAZ IMPORT` button depends on it):

```powershell
& "C:\Program Files\Autodesk\Maya2025\bin\mayapy.exe" -m pip install --user pymel-1.5.0-py2.py3-none-any.whl
```

3. Restart Maya → a **DazToMaya** shelf tab appears → it contains **DAZ IMPORT**.

## 2. Install the DAZ Studio side

Unzip `DazToMaya_DAZ端.zip` and copy `dsp_dzmayabridge.dll` into:

```
C:\Program Files\DAZ 3D\DAZStudio6\plugins\
```

(same folder as `dsp_dzhexagonbridge.dll` and friends; administrator rights required) →
**fully quit DAZ Studio 6 and start it again** → menu item `File → Send To → Daz To Maya` appears.

> This DLL is **DAZ Studio 6 only** (it links against Qt6). Do not put it into DAZ Studio 4.

## 3. Using it

1. Open your character in DAZ Studio and make sure clothing/hair are **parented to the body**.
2. `File → Send To → Daz To Maya`, choose **Skeletal Mesh** (with skeleton; use Static for plain meshes).
3. Tick **Export Morphs** for facial/body sliders; tick **Bake Subdivision** if needed
   (do not exceed level 2 — it can run out of memory).
4. Accept, then switch to Maya when prompted.
5. In Maya, click the **DazToMaya** shelf tab → **DAZ IMPORT** → **Auto-Import**.
6. Morph sliders live on the `Morphs` node in the Outliner → Attribute Editor →
   **Extra Attributes**.

## 4. Troubleshooting

| Symptom | Fix |
|---|---|
| No DazToMaya shelf tab in Maya | Make sure `DazToMaya.mod` sits **next to** the `DazToMaya` folder; if shelf tabs are hidden, click the shelf cog → Shelf Tabs |
| DAZ IMPORT says PyMEL is missing | Add `PYTHONPATH=%APPDATA%\Python\Python311\site-packages` to `Documents\maya\2025\Maya.env`, restart Maya |
| No Send To menu in DS6 | Check `%APPDATA%\DAZ 3D\Studio6\log.txt`; confirm the DLL is in DS6's `plugins` folder and that no DS4 build is mixed in |
| Model falls apart after import | Clothing was not parented to the body, or you exported as Static Mesh |
| Everything is white | The two ends are on different bridge versions, or the export folder was moved |

To uninstall, just delete the copied files — no system settings were changed.
