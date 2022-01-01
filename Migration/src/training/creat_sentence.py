import os

path = "D:\\workspace\\Pycharm\\Understand_analysis"
os.chdir(path)
path1 = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.3testcase\\"
path2 = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.3trainset\\"
training_set = "training_set_filter.txt"
file1 = open(path1+training_set, "r", encoding='utf-8')
file2 = open(path2+training_set, "r", encoding='utf-8')
wr = open("sentence.txt", "w", encoding='utf-8')
lines1 = file1.readlines()
lines2 = file2.readlines()
for line in lines1:
    tmps = line.split("\t")
    wr.write(tmps[0].rstrip()+"\n")
    wr.write(tmps[1].rstrip()+"\n")
for line in lines2:
    tmps = line.split("\t")
    wr.write(tmps[0].rstrip()+"\n")
    wr.write(tmps[1].rstrip()+"\n")
wr.close()