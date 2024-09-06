@echo off
REM 创建一个新的conda环境
conda create --name py39_env python=3.9 -y

REM 激活新创建的环境
conda activate py39_env

REM 使用清华镜像源安装项目依赖
pip install --index-url https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt

echo Environment setup completed.
pause