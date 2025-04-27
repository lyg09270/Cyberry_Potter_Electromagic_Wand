import os
import re
from nnom import * # type: ignore
import numpy as np
import tensorflow as tf # type: ignore
from tensorflow.keras.models import load_model, save_model # type: ignore
from tensorflow.keras.preprocessing.sequence import pad_sequences # type: ignore
from tensorflow.keras import * # type: ignore
from tensorflow.keras.layers import * # type: ignore
from tensorflow.keras.regularizers import l2 # type: ignore

from sklearn.metrics import classification_report
from sklearn.preprocessing import MinMaxScaler, StandardScaler

# 动作分类名
motion_names = ['RightAngle', 'SharpAngle', 'Lightning', 'Triangle', 'Letter_h', 'letter_R', 'letter_W', 'letter_phi', 'Circle', 'UpAndDown', 'Horn', 'Wave', 'NoMotion']

# 定义目录路径
DEF_SAVE_TO_PATH = './TrainingData_8_23'
DEF_MODEL_NAME = 'model.h5'
DEF_MODEL_H_NAME = 'weights.h'
DEF_FILE_MAX = 1000
DEF_MAX_TRIALS = 3
#DEF_N_ROWS = 60
DEF_N_ROWS = 150
#DEF_COLUMNS = (0, 1, 2, 3, 4, 5)
DEF_COLUMNS = (3, 4, 5)
#DEF_COLUMNS = (0, 1, 2)

# 文件格式
DEF_FILE_FORMAT = '.txt'
# 文件名分隔符
DEF_FILE_NAME_SEPERATOR = '_'
DEF_BATCH_SIZE = 100
DEF_NUM_EPOCH = 80

# 动作名称到标签的映射
motion_to_label = {name: idx for idx, name in enumerate(motion_names)}

def train(x_train, y_train, x_test, y_test, input_shape=(DEF_N_ROWS, 2), num_classes=len(motion_names), batch_size=DEF_BATCH_SIZE, epochs=DEF_NUM_EPOCH):
    inputs = layers.Input(shape=input_shape) # type: ignore

    x = layers.Conv1D(30, kernel_size=3, strides=3, padding='same')(inputs) # type: ignore
    x = layers.LeakyReLU()(x)# type: ignore
    x = layers.Conv1D(15, kernel_size=3, strides=3, padding='same')(x)# type: ignore
    x = layers.LeakyReLU()(x)# type: ignore
    #x = layers.MaxPooling1D(pool_size=3, strides=3)(x)# type: ignore
    # LSTM 层
    #x = layers.LSTM(5, return_sequences=True)(x)  # type: ignore
   
    x = layers.Flatten()(x)# type: ignore

    # 全连接层
    x = layers.Dense(num_classes)(x) # type: ignore
    x = layers.Dropout(0.3)(x) # type: ignore
    outputs = layers.Softmax()(x) # type: ignore

    model = models.Model(inputs=inputs, outputs=outputs) # type: ignore
    
    # 编译模型
    model.compile(optimizer=optimizers.Adam(), # type: ignore
                  loss=losses.CategoricalCrossentropy(), # type: ignore
                  metrics=['accuracy'])# type: ignore
    
    model.summary()
    
    # Callbacks
    early_stopping = callbacks.EarlyStopping(monitor='val_loss', patience=10)# type: ignore
    checkpoint = callbacks.ModelCheckpoint(DEF_MODEL_NAME, monitor='val_accuracy', save_best_only=True, mode='max')# type: ignore
    
    # 训练模型
    history = model.fit(
        x_train,
        y_train,
        batch_size=batch_size,
        epochs=epochs,
        verbose=2,
        validation_data=(x_test, y_test),
        shuffle=True,
        callbacks=[early_stopping, checkpoint]
    )
    
    # 清除会话
    del model
    tf.keras.backend.clear_session()
    
    return history

# 加载数据集
def load_dataset(root_dir, max_rows=None):
    file_list = []
    labels = []
    for filename in os.listdir(root_dir):
        if filename.endswith(DEF_FILE_FORMAT):
            match = re.match(rf'^([\w]+)_([\d]+){DEF_FILE_FORMAT}$', filename)
            if match:
                motion_name = match.group(1)
                number_str = match.group(2)
                number = int(number_str)
                if 0 <= number <= DEF_FILE_MAX:
                    if motion_name in motion_to_label:
                        file_path = os.path.join(root_dir, filename)
                        # 使用max_rows参数限制读取的行数
                        data = np.loadtxt(file_path, delimiter=' ', usecols=DEF_COLUMNS, max_rows=max_rows)
                        file_list.append(data)
                        labels.append(motion_to_label[motion_name])
                    else:
                        print(f"Motion name not recognized: {filename}")
                else:
                    print(f"Number out of range: {filename}")
            else:
                print(f"Invalid file name format: {filename}")
    return file_list, labels

file_list, labels = load_dataset(DEF_SAVE_TO_PATH, max_rows=DEF_N_ROWS)

# 数据预处理，例如填充序列以达到统一长度
max_len = max([len(x) for x in file_list])  # 找到最长序列的长度
print(f"Max length of sequences: {max_len}")  # 打印max_len的值
file_list_padded = pad_sequences(file_list, maxlen=max_len, dtype='float32', padding='post', value=0)
    
# 转换标签为one-hot编码
labels_one_hot = utils.to_categorical(labels, num_classes=len(motion_names)) # type: ignore

# 计算分割点前，先确保数据集被完全构建
num_elements = len(file_list_padded)

train_size = int(num_elements * 0.8)

# 训练模型并保存最佳模型
best_val_accuracy = 0
best_model = None
for _ in range(DEF_MAX_TRIALS):  # 重复训练DEF_MAX_TRIALS次
    # 先shuffle再分割
    indices = np.arange(num_elements)
    np.random.shuffle(indices)

    # 分割数据集
    train_indices = indices[:train_size]
    test_indices = indices[train_size:]

    # 获取训练和测试数据
    x_train = file_list_padded[train_indices]
    y_train = labels_one_hot[train_indices]
    x_test = file_list_padded[test_indices]
    y_test = labels_one_hot[test_indices]
        
    history = train(x_train, y_train, x_test, y_test, batch_size=DEF_BATCH_SIZE, epochs=DEF_NUM_EPOCH)
    # 加载最佳模型
    model = load_model(DEF_MODEL_NAME)
    
    # 评估模型
    y_pred = model.predict(x_test)
    y_pred_classes = np.argmax(y_pred, axis=1)
    y_true_classes = np.argmax(y_test, axis=1)
    
    # 获取验证集上的准确率
    val_accuracy = history.history['val_accuracy'][-1]
    print(f"Validation Accuracy: {val_accuracy:.4f}")
    
    # 保存最佳模型
    if val_accuracy > best_val_accuracy:
        best_val_accuracy = val_accuracy
        best_model = model

# 从训练数据集中获取一个批次作为校准数据集
x_test_sample = x_test[:100] 

# 使用最佳模型进行最终评估
if best_model is not None:
    y_pred = best_model.predict(x_test)
    y_pred_classes = np.argmax(y_pred, axis=1)
    y_true_classes = np.argmax(y_test, axis=1)
    
    # 确保 target_names 的长度与实际类别数相匹配
    unique_classes = np.unique(np.concatenate((y_true_classes, y_pred_classes)))
    target_names = [motion_names[i] for i in sorted(unique_classes)]

    # 打印每个类别的准确率
    print(classification_report(y_true_classes, y_pred_classes, target_names=target_names))

    # 从训练数据集中获取一个批次作为校准数据集
    x_test_sample = x_test[:100]  # 使用前100个样本作为校准数据集
    
    generate_model(best_model, x_test_sample, format='hwc', name=DEF_MODEL_H_NAME)