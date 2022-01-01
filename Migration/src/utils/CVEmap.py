import os

path = "D:\\workspace\\Pycharm\\20191222-Vulnerability-dataset\\dataset.csv"
database = open(path, "r")
lines = database.readlines()
CVEList = []
commitList = []
for line in lines:
    line = line.rstrip()
    CVE = line.split(",")[0]
    commit = line.split(",")[2]
    CVEList.append(CVE)
    commitList.append(commit)
print(commitList)
gitPath = "J:\\Vulnerability_commit\\"
dirs = os.listdir(gitPath)
wr = open("CVE_map.csv", "w")
for dir in dirs:
    CVE = ""
    index = -1
    diffPath = gitPath+dir+"\\diffs.txt"
    lines = open(diffPath, "r").readlines()
    repoName = lines[0].rstrip()
    commits = lines[1]
    commit1 = commits.split(";")[0].rstrip()
    commit2 = commits.split(";")[1].rstrip()
    while index == -1:
        if commit2 in commitList:
            index = commitList.index(commit2)
            CVE = CVEList[index]
        else:
            commit2 = commit2[0:-1]
    if index == -1:
        raise Exception("error index!")
    wr.write(dir+","+CVE+"\n")



