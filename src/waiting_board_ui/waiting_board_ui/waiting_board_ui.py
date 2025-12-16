import os
import subprocess

def main():
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    app_path = os.path.join(pkg_dir, "web_app.py")

    subprocess.run([
        "streamlit", "run", app_path,
        "--server.address", "0.0.0.0",
        "--server.port", "8501"
    ])
