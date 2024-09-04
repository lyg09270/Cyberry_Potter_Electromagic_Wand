import numpy as np
import os
import re
from autokeras import StructuredDataClassifier
from tensorflow.keras.utils import to_categorical
from sklearn.model_selection import train_test_split

# 动作分类名
motion_names = ['RightAngle', 'SharpAngle', 'Lightning', 'Triangle', 'Letter_h', 'letter_R', 'letter_W', 'letter_phi', 'Circle', 'UpAndDown', 'Horn', 'Wave', 'NoMotion']

# 定义目录路径
DEF_SAVE_TO_PATH = './TraningData_9_4/'
DEF_MODEL_NAME = 'model.h5'
DEF_MODEL_H_NAME = 'weights.h'
DEF_FILE_MAX = 100
DEF_N_ROWS = 150
DEF_COLUMNS = (3, 4, 5)

# 动作名称到标签的映射
motion_to_label = {name: idx for idx, name in enumerate(motion_names)}

# 加载数据集
def load_dataset(root_dir, max_rows=None):
    file_list = []
    labels = []
    for filename in os.listdir(root_dir):
        if filename.endswith('.txt'):
            match = re.match(r'^([\w]+)_([\d]+)\.txt$', filename)
            if match:
                motion_name = match.group(1)
                number_str = match.group(2)
                number = int(number_str)
                if 0 <= number <= DEF_FILE_MAX:
                    if motion_name in motion_to_label:
                        file_path = os.path.join(root_dir, filename)
                        # 使用max_rows参数限制读取的行数
                        try:
                            data = np.loadtxt(file_path, delimiter=' ', usecols=DEF_COLUMNS, max_rows=max_rows)
                            file_list.append(data)
                            labels.append(motion_to_label[motion_name])
                        except Exception as e:
                            print(f"Error loading file {filename}: {e}")
                    else:
                        print(f"Motion name not recognized: {filename}")
                else:
                    print(f"Number out of range: {filename}")
            else:
                print(f"Invalid file name format: {filename}")
    return file_list, labels

def preprocess_data(file_list, labels):
    X = np.array([np.array(file) for file in file_list])
    y = np.array(labels)
    y = to_categorical(y, num_classes=len(motion_names))
    return X, y

def train_with_autokeras(X_train, y_train, X_test, y_test, max_trials=10, epochs=200, overwrite=False):
    # 初始化AutoKeras分类器
    clf = StructuredDataClassifier(max_trials=max_trials, overwrite=overwrite)
    
    # 训练模型
    clf.fit(X_train, y_train, epochs=epochs)
    
    # 评估模型
    accuracy = clf.evaluate(X_test, y_test)
    print('Accuracy:', accuracy)
    
    # 显示模型结构
    keras_model = clf.export_keras_model()
    keras_model.summary()
    
    return clf

# 加载数据集
file_list, labels = load_dataset(DEF_SAVE_TO_PATH, max_rows=DEF_N_ROWS)
X, y = preprocess_data(file_list, labels)

# 划分数据集
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 使用AutoKeras训练模型
clf = train_with_autokeras(X_train, y_train, X_test, y_test)

# 如果需要保存模型
# clf.export_autokeras_model('model.h5')
# clf.export_keras_model('model_keras.h5')