@echo off
setlocal EnableDelayedExpansion

REM 创建一个新的conda环境
echo Creating conda environment...
conda create --name py39_env python=3.9 -y
if %errorlevel% neq 0 (
    echo Failed to create conda environment.
    pause
    exit /b
)

REM 激活新创建的环境
echo Activating conda environment...
conda activate py39_env
if %errorlevel% neq 0 (
    echo Failed to activate conda environment.
    pause
    exit /b
)

REM 使用清华镜像源安装项目依赖
echo Installing project dependencies using Tsinghua mirror...
pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
if %errorlevel% neq 0 (
    echo Failed to install dependencies.
    pause
    exit /b
)

echo Environment setup completed.
pause