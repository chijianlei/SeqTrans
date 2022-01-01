import os
import shutil
import sys

CVE_path = "D:\\workspace\\Pycharm\\20191222-Vulnerability-dataset\\CVE_2018.csv"
cp_root1 = "I:\\20210714-Srqtrans_testcase\\Vulnerability_trainset\\"
cp_root2 = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase\\"
output_path = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.2"
seq_path1 = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.3trainset\\seqs"
seq_path2 = "D:\\workspace\\Pycharm\\Understand_analysis\\20211130-RQ1.3testcase\\seqs"
cp_dirs1 = os.listdir(cp_root1)
cp_dirs2 = os.listdir(cp_root2)
cp_dict = {}
for file in cp_dirs1:
    diff_path = os.path.join(cp_root1, file, "diffs.txt")
    diff_file = open(diff_path, "r", encoding="utf-8")
    lines = diff_file.readlines()
    commit_line = lines[1]
    cp_dict[file] = commit_line
for file in cp_dirs2:
    diff_path = os.path.join(cp_root2, file, "diffs.txt")
    diff_file = open(diff_path, "r", encoding="utf-8")
    lines = diff_file.readlines()
    commit_line = lines[1]
    cp_dict[file] = commit_line

chrono_list = []
error_list = []
CVE_file = open(CVE_path, "r", encoding="utf-8")
for line in CVE_file.readlines():
    commit = line.split(",")[1].rstrip()
    print(commit)
    find = False
    for file, commit_line in cp_dict.items():
        if commit in commit_line:
            find = True
            chrono_list.append(file)
            print(file)
            break

print(len(chrono_list))
test_path = os.path.join(output_path, "test_seq")
if not os.path.exists(test_path):
    os.makedirs(test_path)
for cp_name in chrono_list:
    seq_files1 = os.listdir(seq_path1)
    for seq_file in seq_files1:
        cpFile_name = seq_file.split('.')[0].split('_')[0]
        if cp_name.rstrip() == cpFile_name:
            print(seq_file)
            seq_path = os.path.join(seq_path1, seq_file)
            try:
                shutil.copy(seq_path, test_path)
            except IOError as e:
                print("Unable to copy file. %s" % e)
            except:
                print("Unexpected error:", sys.exc_info())
    seq_files2 = os.listdir(seq_path2)
    for seq_file in seq_files2:
        cpFile_name = seq_file.split('.')[0].split('_')[0]
        if cp_name.rstrip() == cpFile_name:
            seq_path = os.path.join(seq_path2, seq_file)
            try:
                shutil.copy(seq_path, test_path)
            except IOError as e:
                print("Unable to copy file. %s" % e)
            except:
                print("Unexpected error:", sys.exc_info())

train_path = os.path.join(output_path, "train_seq")
if not os.path.exists(train_path):
    os.makedirs(train_path)
seq_files1 = os.listdir(seq_path1)
for seq_file in seq_files1:
    cpFile_name = seq_file.split('.')[0].split('_')[0]
    notin_chrono = True
    for cp_name in chrono_list:
        if cp_name.rstrip() == cpFile_name:
            notin_chrono = False
            break
    if notin_chrono is True:
        seq_path = os.path.join(seq_path1, seq_file)
        try:
            shutil.copy(seq_path, train_path)
        except IOError as e:
            print("Unable to copy file. %s" % e)
        except:
            print("Unexpected error:", sys.exc_info())
for seq_file in seq_files2:
    cpFile_name = seq_file.split('.')[0].split('_')[0]
    notin_chrono = True
    for cp_name in chrono_list:
        if cp_name.rstrip() == cpFile_name:
            notin_chrono = False
            break
    if notin_chrono is True:
        seq_path = os.path.join(seq_path2, seq_file)
        try:
            shutil.copy(seq_path, train_path)
        except IOError as e:
            print("Unable to copy file. %s" % e)
        except:
            print("Unexpected error:", sys.exc_info())







