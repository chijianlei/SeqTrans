import io
import numpy as np

output = 'src-train-1.txt'
general_path = 'D:\\workspace\\Pycharm\\Understand_analysis\\tmp\\backup_tufanodataset\\'+output
specific_path='D:\\workspace\\Pycharm\\Understand_analysis\\tmp\\backup_singleline\\'+output


general_file = open(general_path, 'r', encoding='utf-8')
general_list = general_file.readlines()
specific_file = open(specific_path, 'r', encoding='utf-8')
specific_list = specific_file.readlines()
s_len = len(specific_list)
N = range(int(len(general_list)/2))
random = np.random.choice(N, size=s_len, replace=False)
output_file = open(output, 'w', encoding='utf-8')
for num in random:
    choice = general_list[num]
    output_file.write(choice)
for line in specific_list:
    output_file.write(line)
output_file.close()
