a = Analysis(["simrate_control.py"])

pyz = PYZ(a.pure, a.zipped_data)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="simrate_control",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=["ucrtbase.dll", "vcruntime140.dll"],
    runtime_tmpdir=None,
    console=True,
)