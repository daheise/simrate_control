# -*- mode: python ; coding: utf-8 -*-
import sys

block_cipher = None
scripts= ['simrate_control.py']

def find_file_by_name(name, paths):
    for path in sys.path:
        for root, dirs, files in os.walk(path):
            if name in files:
                return os.path.join(root, name)

a = Analysis(['simrate_control.py'],
             pathex=[SPECPATH],
             binaries=[(find_file_by_name('SimConnect.dll', [SPECPATH] + sys.path), './SimConnect/')],
             datas=[('config.ini', './'), ('README.md', './'), ('LICENSE', './')],
             hiddenimports=['pyttsx3.drivers','pyttsx3.drivers.sapi5'],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)

pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts,
          [],
          exclude_binaries=True,
          name='simrate_control',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          console=True )

coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               upx_exclude=[],
               name='simrate_control')
