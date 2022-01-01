import os

path = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.2\\train\\"
os.chdir(path)
training_set = "training_set_final.txt"
file = open(training_set, "r", encoding='utf-8')
src_file = open("src-train-1.txt", "w", encoding='utf-8')
dst_file = open("tgt-train-1.txt", "w", encoding='utf-8')
lines = file.readlines()
for line in lines:
    tmps = line.split("\t")
    src_file.write(tmps[0].rstrip()+"\n")
    dst_file.write(tmps[1].rstrip()+"\n")
