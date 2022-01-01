import os


def openCP(analysePath, cpName):
    srcPath = analysePath+cpName+"_src.txt"
    dstPath = analysePath+cpName+"_dst.txt"
    outPath = "training_set_final.txt"
    outPath1 = "training_location_final.txt"
    diffPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_trainset\\" + cpName + "\\diffs.txt"
    srcFile = open(srcPath, "r", encoding='utf-8')
    dstFile = open(dstPath, "r", encoding='utf-8')
    diffFile = open(diffPath, "r")
    wr = open(outPath, "a", encoding='utf-8')
    wr1 = open(outPath1, "a", encoding='utf-8')
    src_lines = srcFile.readlines()
    dst_lines = dstFile.readlines()
    repoName = diffFile.readlines()[0].rstrip()
    print(srcPath, dstPath)
    count = 0
    lineNum = 0
    diff_list = []
    location_list = []
    count_dic = {}
    for src_line in src_lines:
        print(src_line)
        src_line = src_line.lstrip()
        src_line = src_line.rstrip()
        # src_line = src_line.rstrip()+" "+cpName+"_src:"+str(lineNum+1)
        print(src_line)
        if src_line == "------":
            lineNum += 1
            continue
        else:
            dst_line = dst_lines[lineNum]
            if dst_line.rstrip() == "":
                continue
            dst_line = dst_line.lstrip()
            dst_line = dst_line.rstrip()
            token_num = len(src_line.split(" "))+len(dst_line.split(" "))
            if count_dic.get(token_num) is None:
                count_dic[token_num] = 1
            else:
                count_dic[token_num] = count_dic[token_num] +1
            # dst_line = dst_line.rstrip()+" "+cpName+"_src:"+str(lineNum+1)
            if src_line != dst_line:
                seq = src_line + "\t" + dst_line
                location = cpName + ":" + str(lineNum + 1)
                wr.write(seq+"\n")
                wr1.write(location+"\n")
                if "\t" not in seq.rstrip():
                    print(srcPath, src_line+"->"+dst_line+","+str(lineNum+1))
                    raise Exception("error")
                diff_list.append(seq)
                location_list.append(location)
                count += 1
            lineNum += 1
    return count, diff_list, location_list, count_dic, repoName


def search_statement(analysePath, cpName, string):
    srcPath = analysePath + cpName + "_src.txt"
    dstPath = analysePath + cpName + "_dst.txt"
    srcFile = open(srcPath, "r", encoding='UTF-8')
    dstFile = open(dstPath, "r", encoding='UTF-8')
    src_lines = srcFile.readlines()
    dst_lines = dstFile.readlines()
    lineNum = 0
    for src_line in src_lines:
        src_line = src_line.rstrip()
        if src_line == "------":
            lineNum += 1
            continue
        else:
            dst_line = dst_lines[lineNum]
            dst_line = dst_line.rstrip()
            if src_line == string:
                return srcPath


def list_filter(diff_list, location_list):
    diff_list_filter = []
    location_list_filter = []
    count = 0
    for seq in diff_list:
        if seq not in diff_list_filter:
            diff_list_filter.append(seq)
            location = location_list[count]
            location_list_filter.append(location)
        count += 1
    return diff_list_filter, location_list_filter


os.chdir("D:\\workspace\\Pycharm\\Understand_analysis")
analysePath = "seqs\\"
outPath = "training_set_final.txt"
outPath1 = "training_location_filter.txt"
if os.path.exists(outPath):
    os.remove(outPath)
if os.path.exists(outPath1):
    os.remove(outPath1)
files = os.listdir(analysePath)
count = 0
total_list = []
total_location_list = []
count_total_dic = {}
repo_count_dic = {}
repo_diff_dic = {}
used_CPlist = []
for file in files:
    cpName = file[0:len(file)-8]
    if cpName in used_CPlist:
        continue
    num, diff_list, location_list, count_dic, repoName = openCP(analysePath, cpName)
    used_CPlist.append(cpName)
    if repoName not in repo_count_dic.keys():
        repo_count_dic[repoName] = num
        repo_diff_dic[repoName] = diff_list
    else:
        repo_count_dic[repoName] += num
        repo_diff_dic[repoName] += diff_list

    diff_list, location_list = list_filter(diff_list, location_list)  # delete duplicates

    total_list += diff_list
    total_location_list += location_list
    for key, value in count_dic.items():
        if count_total_dic.get(key) is None:
            count_total_dic[key] = 1
        else:
            count_total_dic[key] += 1
    fileName = search_statement(analysePath, cpName, "assertEquals ( var . host , \"\" , \"\" )")
    if fileName is not None:
        print(fileName)
    count += num
print("totalnum: ", count)

total_dic = {}
# total_list, total_location_list = list_filter(total_list, total_location_list)  # 全局去重
wr = open("training_set_filter.txt", "w", encoding='utf-8')
wr1 = open("training_location_filter.txt", "w", encoding='utf-8')
count = 0
for seq in total_list:  # 保存为dict
    if "\t" not in seq.rstrip():
        raise Exception("error")
    # if len(seq) > 200:
    #     continue
    # if "{" in seq:
    #     src = seq.split("\t")[0]
    #     dst = seq.split("\t")[1]
    #     print(src)
    #     print(dst)
    #     print("-----------")
    total_dic[seq] = total_list.count(seq)
    wr.write(seq+"\n")
    location = total_location_list[count]
    wr1.write(location+"\n")
    count += 1
total_dic = sorted(total_dic.items(), key=lambda x: x[1], reverse=True)  # 降序
# total_dic = sorted(total_dic.items(), key=lambda x: x[1])  # 升序
# print(total_dic)

count_total_dic = sorted(count_total_dic.items(), key=lambda x: x[0])  # 升序
wr = open("frequency.csv", "w", encoding='utf-8')
count_up200 = 0
for key, value in count_total_dic:
    if key <= 200:
        wr.write(str(key) + "," + str(value) + "\n")
    else:
        count_up200 += value
wr.write(str(201) + "," + str(count_up200) + "\n")

print(repo_diff_dic)
dir = "repo_splits\\"
if not os.path.exists(dir):
    os.makedirs(dir)
for key, value in repo_diff_dic.items():
    fileName = dir+key+".txt"
    wr = open(fileName, "w")
    value = set(value)
    for seq in value:
        seq = seq.rstrip()
        wr.write(seq+"\n")

wr = open("repo_count.txt", "w", encoding='utf-8')
repo_count_dic = sorted(repo_count_dic.items(), key=lambda x: x[1], reverse=True)  # 降序
for key, value in repo_count_dic:
    wr.write(key+","+str(value)+"\n")


