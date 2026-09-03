@echo off
call D:\Miniconda\Scripts\activate.bat wham_gmr
set OUTPUT_ROOT=output/linglong2_run
set ROBOT=linglong2
set RECORD_GMRVIDEO=1
set RECORD_WHAMVIDEO=0
set VIDEO=examples/dataset_video.mp4

:: LingLong 2.0 optimized parameters
set GMR_MAX_ITER=20
set GMR_WORKER_QUEUE_SIZE=64

powershell -NoProfile -ExecutionPolicy Bypass -File run.ps1