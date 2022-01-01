import getopt
import ntpath
import shutil
import subprocess
import logging
import sys
import os
import understand
import time
import ctypes


def main(argv):
    input_file = ''
    num_file = ''
    var_file = ''
    output_file = ''
    try:
        opts, args = getopt.getopt(argv, "i:n:v:o:", ["ifile=", "nfile=", "vfile=", "ofile="])
    except getopt.GetoptError:
        print('test.py -i <inputfile> -n <numfile>  -v <varfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-i", "--ifile"):
            input_file = arg
        elif opt in ("-n", "--nfile"):
            num_file = arg
        elif opt in ("-v", "--vfile"):
            var_file = arg
        elif opt in ("-o", "--ofile"):
            output_file = arg

    print('输入的文件为：', input_file)
    print('输出的文件为：', output_file)


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


def normalization(token, defLine, recover_dic):
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
        replace = "var" + str(index + 1)
        recover_dic[token] = replace
        token = replace

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


def stmt_extraction(db, exact_longName, start_line, end_line, start_col, end_col, defLine, num_list, recover_dic):
    findFile = False
    begin = False  # in some cases, statements will begin with irrelevant lexeme
    statement = ""
    print(start_line, end_line, start_col, end_col)
    for file in db.ents("File"):
        if exact_longName in file.longname():
            print("find file:", file.longname())
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
                        recover_dic[lexeme.text()] = text
                    else:
                        text = "num" + str(len(num_list) + 1)  # replace Literal
                        num_list.append(lexeme.text())
                        recover_dic[lexeme.text()] = text

                if lexeme.token() == "String":
                    text = "str"  # replace String literal
                    recover_dic[lexeme.text()] = text
                text = normalization(text, defLine, recover_dic)  # normalization
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


def abstract(input_path, num_path, var_path, output_path):
    print("understand version:", understand.version())
    # dataPath = "D:\\workspace\\Pycharm\\Understand_analysis\\data_num\\"
    # defPath = "D:\\workspace\\Pycharm\\Understand_analysis\\data_var\\"
    # outputPath = "D:\\workspace\\Pycharm\\Understand_analysis\\seqs\\"
    # dbRoot = "I:\\UDB_backup_testcase\\"
    # cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testcase\\"
    if os.path.exists(input_path) is False:
        raise Exception("check input File!")
    if os.path.exists(num_path) is False:
        raise Exception("check num File!")
    if os.path.exists(var_path) is False:
        raise Exception("check var File!")

    # files = os.listdir(dataPath)
    # def_files = os.listdir(defPath)
    # existFiles = os.listdir(outputPath)
    UnicodeErrorList = []
    exceptList = []
    srcName = ntpath.basename(input_path)
    projectPath = os.path.dirname(input_path)
    src_tuple_list = []
    src_defPath = var_path
    dbPath = projectPath
    print('Analyze:', srcName)

    old_path = os.path.join(dbPath, 'old.udb')
    print(old_path)
    if os.path.exists(dbPath) is False:
        os.makedirs(dbPath)
    if not os.path.exists(old_path):
        create_udb(old_path, "java", projectPath)
    try:
        db = understand.open(old_path)
    except understand.UnderstandError:
        os.remove(old_path)
        create_udb(old_path, "java", projectPath)
        db = understand.open(old_path)

    # print(len(db.ents("File")))
    if len(db.ents("File")) == 0:
        raise Exception('Error dbFile!')

    txt = open(num_path, "r")
    txt_srcDef = open(var_path, "r")
    lines = txt.readlines()
    try:
        lines_srcDef = txt_srcDef.readlines()
    except UnicodeDecodeError:
        print('Decode Error!')

    fileName = ""
    localtime = time.asctime(time.localtime(time.time()))
    print('\033[1;32m' + "analyse src:" + localtime + '\033[0m')
    src = open(output_path+"abstract.txt", "w", encoding='utf-8')
    recover_dic = {}
    # open(output_path + "recover.txt", "w", encoding='utf-8')
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
        exact_name = ""
        num_list_src = []
        begin_index = 0
        # try:
        #     begin_index = tmps.index("src")
        # except ValueError:
        #     try:
        #         begin_index = tmps.index("java")
        #     except ValueError:
        #         try:
        #             begin_index = tmps.index("org")
        #         except ValueError:
        #             begin_index = tmps.index("javax")
        tmps = tmps[begin_index:len(tmps)]
        exact_name = tmps[-1]
        print("exactName:", exact_name)

        for file in db.ents("File"):
            print("file name: "+file.longname())
            if exact_name in file.longname():
                print("find file:", exact_name)

        try:
            print(line)
            startLine_src = int(line.split(";")[1].split("->")[0].split(",")[0])
            endLine_src = int(line.split(";")[1].split("->")[0].split(",")[1])
            startCol_src = int(line.split(";")[1].split("->")[0].split(",")[2])
            endCol_src = int(line.split(";")[1].split("->")[0].split(",")[3])
            # in some cases it will throw list index out of range, continue
            fullText, findFile = stmt_extraction(db, exact_name, startLine_src, endLine_src, startCol_src,
                                                 endCol_src, defLine_src, num_list_src, recover_dic)
            # src.write(fullText+"\n")
            src_tuple_list.append((fullText, exact_name, defLine_src, num_list_src))
            print("-----------")
            if findFile is False:
                raise Exception("File is not searched!", exact_name)
        except Exception as e:
            print(e)
        lineNum += 1

    localtime = time.asctime(time.localtime(time.time()))
    print('\033[1;32m' + "analyse def:" + localtime + '\033[0m')
    lineNum = 0
    for src_tuple in src_tuple_list:
        src_stmt = src_tuple[0]
        exact_longName_src = src_tuple[1]
        defLine_src = src_tuple[2]
        num_list_src = src_tuple[3]
        print("defLine:", defLine_src)
        src_stmt = "<START_VUL> " + src_stmt.rstrip() + " <END_VUL>"  # add the location labels
        tmps = defLine_src.split(";")
        for tmp in tmps:
            if tmp.__contains__(","):  # def location
                startLine_src = int(tmp.split(",")[0])
                endLine_src = int(tmp.split(",")[1])
                startCol_src = int(tmp.split(",")[2])
                endCol_src = int(tmp.split(",")[3])
                def_srcLine, findFile = stmt_extraction(db, exact_longName_src, startLine_src, endLine_src,
                                                        startCol_src, endCol_src, defLine_src, num_list_src, recover_dic)
                if findFile is False:
                    raise Exception("File is not searched!", exact_longName_src)
                src_stmt = def_srcLine.strip() + " ; " + src_stmt
        src.write(src_stmt + "\n")
        lineNum += 1
    src.close()

    recover_file = open(output_path + "recover.txt", "w", encoding='utf-8')
    for key, value in recover_dic.items():
        recover_file.write(key+"->"+value+";")
    recover_file.close()


# if __name__ == "__main__":
#     main(sys.argv[1:])
root_path = "D:\\workspace\\eclipse2018\\Migration\\test\\"
input_path = root_path+"AccountInstance.java"
num_path = root_path+"src-num.txt"
var_path = root_path+"src-val.txt"
output_path = root_path
abstract(input_path, num_path, var_path, output_path)