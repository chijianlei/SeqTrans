import os


path = 'D:\\workspace\\Pycharm\\Understand_analysis\\tmp\\backup_singleline'
file = open(path+'\\training_final_tufano.txt', 'r', encoding='utf-8')
file_new = open(path+'\\training_final_tufano_filter100.txt', 'w', encoding='utf-8')
lines = file.readlines()
print(lines.__len__())
limited_length = 100
tmp = ''
for line in lines:
    src = line.split('\t')[0]
    tgt = line.split('\t')[1]
    tokens_src = src.split(' ')
    tokens_tgt = tgt.split(' ')
    if len(tokens_src)==0 or len(tokens_tgt)==0:
        print(line)
    if len(tokens_src) <= limited_length and len(tokens_tgt)<=limited_length:
        file_new.write(line)



