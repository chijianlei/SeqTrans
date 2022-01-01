'''
Created on 2019年9月13日

@author: yu
'''
from sklearn.model_selection import train_test_split
import os


def read_text_data(data_file):
    if 'txt' == data_file.split(".")[-1]:
        handle = open(data_file, 'r', encoding='UTF-8')
        lines = handle.readlines()
        result_list = []
        for each_line in lines:
            result_list.append(each_line)
    return result_list


experiment_id = 1
dataPath = "D:\\workspace\\Pycharm\\Understand_analysis\\tmp\\backup_singletest\\"
text_data = read_text_data(dataPath+'training_set_filter.txt')

sentence = open(dataPath + 'sentence.txt', 'w', encoding='UTF-8')
for line in text_data:
    src = line.split('\t')[0]
    dst = line.split('\t')[1]
    sentence.write(src+'\n')
    sentence.write(dst)
sentence.close()

train_data, test_data = train_test_split(text_data, test_size=0.1, random_state=10)

src_train = open(dataPath + 'src-train-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')
tgt_train = open(dataPath + 'tgt-train-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')

src_val = open(dataPath + 'src-val-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')
tgt_val = open(dataPath + 'tgt-val-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')

src_test = open(dataPath + 'src-test-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')
tgt_test = open(dataPath + 'tgt-test-' + str(experiment_id) + '.txt', 'w', encoding='UTF-8')

#     for each_pair in train_data:
for each_pair in train_data:
    src_train.write(each_pair.split('\t')[0] + '\n')
    tgt_train.write(each_pair.split('\t')[1])
#     for each_pair in val_data:
for each_pair in test_data:
    src_val.write(each_pair.split('\t')[0] + '\n')
    tgt_val.write(each_pair.split('\t')[1])
for each_pair in test_data:
    src_test.write(each_pair.split('\t')[0] + '\n')
    tgt_test.write(each_pair.split('\t')[1])

src_train.close()
tgt_train.close()
src_val.close()
tgt_val.close()
src_test.close()
tgt_test.close()