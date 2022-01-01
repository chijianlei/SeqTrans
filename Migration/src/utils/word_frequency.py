import os

path = 'training_set_final.txt'
dataset = open(path, 'r', encoding='UTF-8')
frequency_dic = {}
output = open('frequency.txt', 'w', encoding='UTF-8')
for line in dataset.readlines():
    tokens = line.replace('\t', ' ').split(' ')
    for token in tokens:
        if token not in frequency_dic:
            frequency_dic[token] = 1
        else:
            frequency_dic[token] += 1

print(len(frequency_dic))
frequency_dic = sorted(frequency_dic.items(), key=lambda x: x[1], reverse=True)

for token, frequency in frequency_dic:
    output.write(token+' '+str(frequency)+'\n')
