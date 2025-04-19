import os
import shutil
import re

def get_max_number_in_folder(folder):
    max_number = 0
    for file_name in os.listdir(folder):
        match = re.search(r'(\d+)', file_name)
        if match:
            number = int(match.group(1))
            max_number = max(max_number, number)
    return max_number

def update_filename(file_name, increment):
    # 使用正则表达式查找文件名中的数字并增加增量
    match = re.search(r'(\d+)', file_name)
    if match:
        # 提取数字并进行增量处理
        number = int(match.group(1))
        new_number = number + increment
        # 替换原文件名中的数字部分
        new_file_name = file_name.replace(str(number), str(new_number))
        return new_file_name
    return file_name

def copy_files_with_updated_names(source_folder, target_folder):
    # 获取目标文件夹中的最大数字
    max_number_in_target = get_max_number_in_folder(target_folder)
    
    # 遍历源文件夹中的所有文件
    for file_name in os.listdir(source_folder):
        source_file = os.path.join(source_folder, file_name)
        if os.path.isfile(source_file):
            # 更新文件名
            new_file_name = update_filename(file_name, max_number_in_target)
            target_file = os.path.join(target_folder, new_file_name)
            # 复制文件到目标文件夹
            shutil.copy(source_file, target_file)
            print(f"文件 '{file_name}' 被复制为 '{new_file_name}' 到目标文件夹")

if __name__ == "__main__":
    # 获取用户输入
    source_folder = input("请输入源文件夹路径：")
    target_folder = input("请输入目标文件夹路径：")
    
    # 调用函数复制文件
    copy_files_with_updated_names(source_folder, target_folder)
