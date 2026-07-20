# -*- mode: python ; coding: utf-8 -*-
# Build : pyinstaller pguard_navigator.spec

from PyInstaller.utils.hooks import collect_all, collect_data_files

block_cipher = None

datas = [("map.osm", ".")]

for package in ("folium", "branca", "xyzservices"):
    pkg_datas, pkg_binaries, pkg_hiddenimports = collect_all(package)
    datas += pkg_datas

hiddenimports = [
    "route_engine",
    "networkx",
    "PyQt6.QtWebEngineWidgets",
    "PyQt6.QtWebChannel",
    "PyQt6.QtWebEngineCore",
]

for package in ("folium", "branca", "xyzservices"):
    _, _, pkg_hiddenimports = collect_all(package)
    hiddenimports += pkg_hiddenimports

a = Analysis(
    ["pguard_navigator.py"],
    pathex=[],
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["osmnx", "selenium", "matplotlib", "pandas", "scipy"],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="PGuardNavigator",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="PGuardNavigator",
)
