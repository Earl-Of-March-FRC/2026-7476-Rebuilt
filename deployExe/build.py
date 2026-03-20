import subprocess
import sys
import os
from PIL import Image

# CONFIG
BAT_FILE = "deploy.bat"
IMAGE_FILE = "better_icon.jpg"
EXE_NAME = "RobotDeploy"

# Get the directory where build.py lives
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Convert image to .ico
print("Converting image to .ico...")
img = Image.open(os.path.join(SCRIPT_DIR, IMAGE_FILE))
img = img.convert("RGBA")
img.save(os.path.join(SCRIPT_DIR, "icon.ico"), format="ICO", sizes=[(256, 256)])
print("Done.")

# Write a Python wrapper that calls the bat file
wrapper = f"""
import subprocess
import os
import sys
import tempfile
import shutil

if getattr(sys, 'frozen', False):
    base_path = sys._MEIPASS
else:
    base_path = os.path.dirname(os.path.abspath(__file__))

bat_source = os.path.join(base_path, "{BAT_FILE}")
temp_dir = tempfile.mkdtemp()
bat_temp = os.path.join(temp_dir, "{BAT_FILE}")
shutil.copy2(bat_source, bat_temp)

subprocess.run(bat_temp, shell=True)

shutil.rmtree(temp_dir, ignore_errors=True)
"""

with open(os.path.join(SCRIPT_DIR, "runner.py"), "w") as f:
    f.write(wrapper)

# Run PyInstaller
print("Building .exe...")
subprocess.run([
    sys.executable, "-m", "PyInstaller",
    "--onefile",
    "--icon=" + os.path.join(SCRIPT_DIR, "icon.ico"),
    f"--name={EXE_NAME}",
    "--add-data", os.path.join(SCRIPT_DIR, BAT_FILE) + ";.",
    "--distpath", os.path.join(SCRIPT_DIR, "dist"),
    "--workpath", os.path.join(SCRIPT_DIR, "build"),
    "--specpath", SCRIPT_DIR,
    os.path.join(SCRIPT_DIR, "runner.py")
], check=True)

print("\nDone! Your .exe is in the deployExe/dist/ folder.")