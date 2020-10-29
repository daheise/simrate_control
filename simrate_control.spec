# -*- mode: python ; coding: utf-8 -*-

block_cipher = None
scripts= ['simrate_control.py']

a = Analysis(['simrate_control.py'],
             pathex=[SPECPATH],
             binaries=[(SPECPATH + "/env/Lib/site-packages/SimConnect/SimConnect.dll", './SimConnect/')],
             datas=[],
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
