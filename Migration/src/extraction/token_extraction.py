import shutil
import subprocess
import logging
import sys
import os
import understand
import time
import ctypes


def create_udb(udb_path, language, project_root):
    try:
        output = subprocess.check_output(
            "und create -db {udb_path} -languages {lang} add {project} analyze -all".format(udb_path=udb_path,
                                                                                            lang=language,
                                                                                            project=project_root),
            shell=True)
        logging.info(output)
    except subprocess.CalledProcessError as e:
        logging.exception(e.output)
        logging.fatal("udb creation failed")
        raise Exception


def normalization(token, defLine):
    defs = defLine.split(";")
    if len(defs) == 0:
        return token  # no def exists
    else:
        defs = defs[:-1]  # delete the last ";", it's useless
    count = 1
    def_List = []
    for definition in defs:
        if "->" not in definition:
            count += 1
            continue
        else:
            tmps = definition.split("->")
            var = tmps[0]
            if var not in def_List:
                def_List.append(var)
        count += 1

    if token in def_List:
        index = def_List.index(token)
        token = "var" + str(index + 1)

    return token


def clean(stmt, left_bracket, right_bracket):
    stmt = str(stmt)
    left_bracket_num = stmt.count(left_bracket)
    right_bracket_num = stmt.count(right_bracket)

    if left_bracket_num < right_bracket_num:
        while left_bracket_num != right_bracket_num:
            location = stmt.rindex(right_bracket)
            tmp = list(stmt)
            tmp[location] = ""
            stmt = ''.join(tmp)
            right_bracket_num = stmt.count(right_bracket)
    elif left_bracket_num > right_bracket_num:
        while left_bracket_num != right_bracket_num:
            stmt = stmt + " " + right_bracket
            right_bracket_num = stmt.count(right_bracket)

    return stmt


def stmt_extraction(db, exact_longName, start_line, end_line, start_col, end_col, defLine, num_list):
    findFile = False
    begin = False  # in some cases, statements will begin with irrelevant lexeme
    statement = ""
    print(start_line, end_line, start_col, end_col)
    for file in db.ents("File"):
        if exact_longName in file.longname():
            print("find file:", fileName, file.longname())
            findFile = True
            # if fileName == "AbstractProject.java":
            #     print("test")
            #     lexemes = file.lexer().lexemes(1648, 1648)
            #     for lexeme in lexemes:
            #         print(lexeme.text(), end="")
            #         if lexeme.ent():
            #             print("@", end="")

            lexemes = file.lexer().lexemes(start_line, end_line)  # locate statement
            start_lexeme = file.lexer().lexeme(start_line, start_col - 1)
            # print("begin_lexeme:", start_lexeme.text())
            end_lexeme = file.lexer().lexeme(end_line, end_col - 1)
            print("lexeme_size:", len(lexemes))
            # for lexeme in lexemes:
            #     print(lexeme.text(), end="")
            #     if lexeme.ent():
            #         print("@", end="")
            type_exist = True  # in some cases, lexeme type is none
            start_text = None
            start_token = None
            if start_lexeme is not None:
                start_text = start_lexeme.text()
                start_token = start_lexeme.token()
                # print("start_text:" + start_text)
            else:
                type_exist = False
                print("\033[31mError type!\033[0m")

            count = 0
            size = len(lexemes)
            for lexeme in lexemes:
                count += 1
                if type_exist is True:
                    if (lexeme.text() != start_text or lexeme.token() != start_token) and begin is False:
                        print("check:", lexeme.text(), ";", start_text, ";", lexeme.token(), ";", start_token, ";",
                              begin)
                        print("\033[32mNot equal lexeme!\033[0m")
                        continue
                    elif lexeme.text() == start_text or lexeme.token() == start_token:
                        begin = True  # filter meaningless token before the first meaningful token
                else:
                    continue

                if lexeme.token() == "Comment" or lexeme.token() == "Whitespace" or lexeme.token() == "Newline" \
                        or lexeme.token() == "Indent" or lexeme.token() == "Dedent":
                    continue  # skip meaningless token

                text = str(lexeme.text())
                if lexeme.token() == "Literal":
                    if lexeme.text() in num_list:
                        index = num_list.index(lexeme.text())
                        text = "num" + str(index + 1)  # replace Literal
                    else:
                        text = "num" + str(len(num_list) + 1)  # replace Literal
                        num_list.append(lexeme.text())
                if lexeme.token() == "String":
                    text = "str"  # replace String literal
                text = normalization(text, defLine)  # normalization
                text = text + " "
                if count == size - 1:
                    text = text.replace("}", "")
                    text = text.replace("{", "")
                    text = text.replace(";", "")  # delete { } and ; in the last token
                    # text = text.rstrip()
                statement += text
                # print(text, end="")
            # for lexeme in lexemes:
            #     src1.write(lexeme.text()+"("+lexeme.token()+")")
            print('fullText: ', statement)
            statement = clean(statement, "(", ")")
            statement = clean(statement, "{", "}")
            statement = clean(statement, "<", ">")
            print('cleanText:', statement)

    return statement, findFile


print("understand version:", understand.version())
dataPath = "D:\\workspace\\Pycharm\\Understand_analysis\\data_num\\"
defPath = "D:\\workspace\\Pycharm\\Understand_analysis\\data_var\\"
outputPath = "D:\\workspace\\Pycharm\\Understand_analysis\\seqs\\"
dbRoot = "I:\\UDB_backup_testcase\\"
cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase\\"
gitRoot = "J:\\git_repo\\"
if os.path.exists(outputPath) is False:
    os.makedirs(outputPath)
files = os.listdir(dataPath)
def_files = os.listdir(defPath)
existFiles = os.listdir(outputPath)
UnicodeErrorList = []
exceptList = []
for txtFile in files:
    txtName = txtFile[0:len(txtFile) - 4]
    repoName = txtName.split("_")[0]  # 记得改
    cpName = txtName.split("_")[-1]  # 记得改
    srcName = cpName + "_src.txt"
    # srcName1 = txtName + "_token_src.txt"
    dstName = cpName + "_dst.txt"
    src_tuple_list = []
    dst_tuple_list = []
    # dstName1 = txtName + "_token_dst.txt"

    output_exist = False
    for existFile in existFiles:
        if srcName == existFile:
            print('\033[1;33m' + "Skip " + txtName + '\033[0m')
            output_exist = True
            break
    if output_exist is True:
        continue  # skip existing output file in order to continue the process

    src_defPath = defPath + txtName + "_defs_src.txt"
    dst_defPath = defPath + txtName + "_defs_dst.txt"
    dbPath = dbRoot + txtName
    gitPath = gitRoot + repoName + "\\"
    diffPath = cpRoot + cpName + "\\diffs.txt"
    diffFile = open(diffPath, "r")
    diffLines = diffFile.readlines()
    old_commit = diffLines[1].split(";")[0]
    new_commit = diffLines[1].split(";")[1]
    os.chdir(gitPath)
    print('Analyze:', txtName)

    old_path = dbPath + "\\old.udb"
    print(old_path)
    gitPath = gitRoot + repoName + "\\"
    print(gitPath)
    os.system("git checkout -f " + old_commit)
    if os.path.exists(dbPath) is False:
        os.makedirs(dbPath)
    if not os.path.exists(old_path):
        create_udb(old_path, "java", gitPath)
    try:
        db = understand.open(old_path)
    except understand.UnderstandError:
        os.remove(old_path)
        create_udb(old_path, "java", gitPath)
        db = understand.open(old_path)

    # print(len(db.ents("File")))
    if len(db.ents("File")) == 0:
        print("\033[31mError dbFile!\033[0m")
        continue  # in some cases Understand cannot recognize the diffs,continue
    # for file in db.ents("File"):
    #     print("dbFile:", file.longname())

    txt = open(dataPath + txtFile, "r")
    txt_srcDef = open(src_defPath, "r")
    txt_dstDef = open(dst_defPath, "r")
    lines = txt.readlines()
    try:
        lines_srcDef = txt_srcDef.readlines()
        lines_dstDef = txt_dstDef.readlines()
    except UnicodeDecodeError:
        UnicodeErrorList.append(repoName)  # record UnicodeDecodeError and continue
        continue

    fileName = ""
    localtime = time.asctime(time.localtime(time.time()))
    print('\033[1;32m' + "analyse src:" + localtime + '\033[0m')
    src = open(outputPath + srcName, "w", encoding='utf-8')
    # src1 = open(outputPath+srcName1, "w")
    dst = open(outputPath + dstName, "w", encoding='utf-8')
    # dst1 = open(outputPath+dstName1, "w")

    lineNum = 0
    for line in lines:
        try:
            defLine_src = lines_srcDef[lineNum]  # the defLine_src in the same lineNum
        except IndexError:
            defLine_src = ''  # avoid throwing, var string is null
        print(defLine_src)
        # J:\Vulnerability_commit\cp157\7acfa2fbb86f5d061ed03fdf1a4a8f0876c94cab\prov\src\main\java\org\bouncycastle\jcajce\provider\asymmetric\dh\IESCipher.java;J:\Vulnerability_commit\cp157\9385b0ebd277724b167fe1d1456e3c112112be1f\prov\src\main\java\org\bouncycastle\jcajce\provider\asymmetric\dh\IESCipher.java;56,57,1,23->56,57,1,23
        longName_src = line.split(";")[0]
        print("longName_src:", longName_src)
        tmps = longName_src.split("\\")
        tmpNames = longName_src.split("\\")
        fileName = tmpNames[len(tmpNames) - 1]
        exact_longName = ""
        num_list_src = []
        begin_index = 0
        try:
            begin_index = tmps.index("src")
        except ValueError:
            try:
                begin_index = tmps.index("java")
            except ValueError:
                try:
                    begin_index = tmps.index("org")
                except ValueError:
                    begin_index = tmps.index("javax")
        tmps = tmps[begin_index:len(tmps)]
        for tmp in tmps:
            exact_longName += tmp + "\\"
        exact_longName = exact_longName[:-1]
        print("exactName:", exact_longName)

        try:
            startLine_src = int(line.split(";")[2].split("->")[0].split(",")[0])
            endLine_src = int(line.split(";")[2].split("->")[0].split(",")[1])
            startCol_src = int(line.split(";")[2].split("->")[0].split(",")[2])
            endCol_src = int(line.split(";")[2].split("->")[0].split(",")[3])
            # in some cases it will throw list index out of range, continue
            fullText, findFile = stmt_extraction(db, exact_longName, startLine_src, endLine_src, startCol_src,
                                                 endCol_src, defLine_src, num_list_src)
            # src.write(fullText+"\n")
            src_tuple_list.append((fullText, exact_longName, defLine_src, num_list_src))
            print("-----------")
            if findFile is False:
                raise Exception("File is not searched!", exact_longName)
        except Exception as e:
            print(e)
        lineNum += 1

    os.system("git checkout -f " + new_commit)
    new_path = dbPath + "\\new.udb"
    if not os.path.exists(new_path):
        create_udb(new_path, "java", gitPath)
    try:
        db = understand.open(new_path)
    except understand.UnderstandError:
        os.remove(new_path)
        create_udb(new_path, "java", gitPath)
        db = understand.open(new_path)

    if len(db.ents("File")) == 0:
        print("\033[31mError dbFile!\033[0m")  # in some cases Understand cannot recognize the diffs,continue

    lineNum = 0
    localtime = time.asctime(time.localtime(time.time()))
    print('\033[1;32m' + "analyse dst:" + localtime + '\033[0m')
    for line in lines:
        try:
            defLine_dst = lines_dstDef[lineNum]  # the defLine_dst in the same lineNum
        except IndexError:
            defLine_dst = ''  # avoid throwing, var string is null
        # example:SimpleBindRequestTestCase.java:37,38,1,39->38,39,1,39
        longName_dst = line.split(";")[1]
        print("longName_dst:", longName_dst)
        tmps = longName_dst.split("\\")
        tmpNames = longName_dst.split("\\")
        fileName = tmpNames[len(tmpNames) - 1]
        exact_longName = ""
        num_list_dst = []
        begin_index = 0
        try:
            begin_index = tmps.index("src")
        except ValueError:
            try:
                begin_index = tmps.index("java")
            except ValueError:
                try:
                    begin_index = tmps.index("org")
                except ValueError:
                    begin_index = tmps.index("javax")
        tmps = tmps[begin_index:len(tmps)]
        for tmp in tmps:
            exact_longName += tmp + "\\"
        exact_longName = exact_longName[:-1]
        print("exactName:", exact_longName)

        try:
            startLine_dst = int(line.split(":")[2].split("->")[1].split(",")[0])
            endLine_dst = int(line.split(":")[2].split("->")[1].split(",")[1])
            startCol_dst = int(line.split(":")[2].split("->")[1].split(",")[2])
            endCol_dst = int(line.split(":")[2].split("->")[1].split(",")[3])
            # in some cases it will throw list index out of range, continue
            fullText, findFile = stmt_extraction(db, exact_longName, startLine_dst, endLine_dst, startCol_dst,
                                                 endCol_dst, defLine_dst, num_list_dst)
            # dst.write(fullText+"\n")
            dst_tuple_list.append((fullText, exact_longName, defLine_dst, num_list_dst))
            print("-----------")
            if findFile is False:
                raise Exception("File is not searched!", exact_longName)
        except Exception as e:
            print(e)
        lineNum += 1

    if len(src_tuple_list) != len(dst_tuple_list):
        raise Exception("check the lists!", len(src_tuple_list), len(dst_tuple_list))

    localtime = time.asctime(time.localtime(time.time()))
    print('\033[1;32m' + "analyse def:" + localtime + '\033[0m')
    os.system("git checkout -f " + old_commit)
    db = understand.open(old_path)
    lineNum = 0
    for src_tuple in src_tuple_list:
        dst_tuple = dst_tuple_list[lineNum]
        src_stmt = src_tuple[0]
        dst_stmt = dst_tuple[0]
        exact_longName_src = src_tuple[1]
        defLine_src = src_tuple[2]
        num_list_src = src_tuple[3]
        print("defLine:", defLine_src)
        if src_stmt.strip() == dst_stmt.strip():
            lineNum += 1
            continue
        else:
            src_stmt = "<START_VUL> " + src_stmt.rstrip() + " <END_VUL>"  # add the location labels
            tmps = defLine_src.split(";")
            for tmp in tmps:
                if tmp.__contains__(","):  # def location
                    startLine_src = int(tmp.split(",")[0])
                    endLine_src = int(tmp.split(",")[1])
                    startCol_src = int(tmp.split(",")[2])
                    endCol_src = int(tmp.split(",")[3])
                    def_srcLine, findFile = stmt_extraction(db, exact_longName_src, startLine_src, endLine_src,
                                                            startCol_src, endCol_src, defLine_src, num_list_src)
                    if findFile is False:
                        raise Exception("File is not searched!", exact_longName_src)
                    src_stmt = def_srcLine.strip() + " ; " + src_stmt
            src.write(src_stmt + "\n")
            dst.write(dst_stmt + "\n")
            lineNum += 1

    # os.system("git checkout -f " + new_commit)
    # db = understand.open(new_path)
    # lineNum = 0
    # for dst_tuple in dst_tuple_list:
    #     src_tuple = src_tuple_list[lineNum]
    #     src_stmt = src_tuple[0]
    #     dst_stmt = dst_tuple[0]
    #     exact_longName_dst = dst_tuple[1]
    #     defLine_dst = dst_tuple[2]
    #     print("defLine:", defLine_dst)
    #     if src_stmt.strip() == dst_stmt.strip():
    #         lineNum += 1
    #         continue
    #     else:
    #         tmps = defLine_dst.split(";")
    #         for tmp in tmps:
    #             if tmp.__contains__(","):  # def location
    #                 startLine_dst = int(tmp.split(",")[0])
    #                 endLine_dst = int(tmp.split(",")[1])
    #                 startCol_dst = int(tmp.split(",")[2])
    #                 endCol_dst = int(tmp.split(",")[3])
    #                 def_dstLine, findFile = stmt_extraction(db, exact_longName_dst,
    #                                                         startLine_dst, endLine_dst, startCol_dst, endCol_dst)
    #                 if findFile is False:
    #                     raise Exception("File is not searched!", exact_longName_dst)
    #                 dst_stmt = def_dstLine.strip() + ";" + dst_stmt
    #         dst.write(dst_stmt + "\n")
    #         lineNum += 1

    src.close()
    dst.close()

for error in UnicodeErrorList:
    print('Error:', error)
print('size:', len(UnicodeErrorList))
for error in exceptList:
    print('Except:', error)
print('size:', len(exceptList))
