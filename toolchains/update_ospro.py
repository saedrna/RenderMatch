import os
import shutil

ospro_root = os.path.dirname(__file__)
ospro_root = os.path.join(ospro_root, os.path.pardir)

print(ospro_root)

# 首先把h2o的dll拷贝
files = [
    'h2o_base.dll',
    'h2o_modelpro.dll',
    'h2o_oglpro.dll',
    'h2o_sketchup.dll',
    'osketchpro_frame.dll'
]
src_dir = os.path.join(ospro_root, 'h2o', 'bin')
dst_dir = os.path.join(ospro_root, 'install', 'files',
                       'h2o_osketchpro', 'dlls')

for name in files:
    file = os.path.join(src_dir, name)
    file2 = os.path.join(dst_dir, name)
    shutil.copyfile(file, file2)

# 安装本工程文件
files = [
    'osketchpro_plugin_import.dll',
    'ATFix.exe',
    'PMConvert.exe',
    'SkpConvert.exe',
    'TileMerge.exe',
    'resources/PMConvert_zh_CN.qm',
    'resources/TileMerge_zh_CN.qm'
]
src_dir = os.path.join(ospro_root, 'build', 'bin', 'Release')
dst_dir = os.path.join(ospro_root, 'install', 'files',
                       'h2o_osketchpro', 'dlls')
for name in files:
    file = os.path.join(src_dir, name)
    file2 = os.path.join(dst_dir, name)
    shutil.copyfile(file, file2)

# 安装 ruby 文件
ruby_src = os.path.join(ospro_root, 'build', 'bin', 'Release', 'osketchpro.so')
ruby_dst = os.path.join(ospro_root, 'install', 'files', 'h2o_osketchpro', 'osketchpro.so')
shutil.copyfile(ruby_src, ruby_dst)
