
import subprocess
import os
import sys
import tempfile
import shutil

if getattr(sys, 'frozen', False):
    base_path = sys._MEIPASS
else:
    base_path = os.path.dirname(os.path.abspath(__file__))

bat_source = os.path.join(base_path, "deploy.bat")
temp_dir = tempfile.mkdtemp()
bat_temp = os.path.join(temp_dir, "deploy.bat")
shutil.copy2(bat_source, bat_temp)

subprocess.run(bat_temp, shell=True)

shutil.rmtree(temp_dir, ignore_errors=True)
