import os
from random import choice
import shutil


repo_path = "repo_splits\\"
outpath = "training_set\\"
files = os.listdir(repo_path)
count = 0
for file in files:
    br = open(repo_path+file, "r")
    lines = br.readlines()
    count += len(lines)
print("totalcount:"+str(count))

id = 1
error_count = 0
while id<=10:
    used_list = []
    trainining_set = []
    testing_set = []
    # special_file = repo_path + "jackson-databind.txt"
    # used_list.append("jackson-databind.txt")
    # br = open(special_file, "r")
    # lines = br.readlines()
    # trainining_set = lines
    # print(len(lines))
    # br.close()

    while len(trainining_set) < count * 0.9:
        random_file = choice(files)
        print(random_file)
        if random_file in used_list:
            continue
        br = open(repo_path + random_file, "r")
        lines = br.readlines()
        if len(lines) < 5:
            used_list.append(random_file)
            continue
        trainining_set += lines
        used_list.append(random_file)

    print("used", used_list)
    for file in files:
        if file not in used_list:
            br = open(repo_path + file, "r")
            lines = br.readlines()
            testing_set += lines
            used_list.append(file)

    train_file_src = open(outpath+"src-train-"+str(id)+".txt", "w")
    train_file_tgt = open(outpath+"tgt-train-"+str(id)+".txt", "w")
    for seq in trainining_set:
        seq = seq.rstrip()
        seqs = seq.split("\t")
        if len(seqs) < 2:
            error_count += 1
            continue
        src = seqs[0]
        tgt = seqs[1]
        train_file_src.write(src+"\n")
        train_file_tgt.write(tgt+"\n")
    shutil.copy(outpath+"src-train-"+str(id)+".txt", outpath+"src-val-"+str(id)+".txt")
    shutil.copy(outpath+"tgt-train-"+str(id)+".txt", outpath+"tgt-val-"+str(id)+".txt")
    test_file_src = open(outpath+"src-utils-"+str(id)+".txt", "w")
    test_file_tgt = open(outpath+"tgt-utils-"+str(id)+".txt", "w")
    for seq in testing_set:
        seq = seq.rstrip()
        seqs = seq.split("\t")
        if len(seqs) < 2:
            error_count += 1
            continue
        src = seqs[0]
        tgt = seqs[1]
        test_file_src.write(src+"\n")
        test_file_tgt.write(tgt+"\n")
    id += 1
print(error_count)
