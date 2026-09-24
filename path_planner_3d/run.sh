#!/bin/bash
cd "$(dirname "$0")"

# Путь к conda (важно: это файл conda.sh, а не папка)
source /home/maksim/miniconda3/etc/profile.d/conda.sh
conda activate pg_env

PYOPENGL_PLATFORM=glx QT_QPA_PLATFORM=xcb python main.py
