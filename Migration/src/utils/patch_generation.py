import sys
import os


def main(argv):
    buggy_file_lines = open(argv[0], "r").readlines()
    buggy_line_number = int(argv[1])
    buggy_line = buggy_file_lines[buggy_line_number-1]
    predictions = open(argv[2], "r").readlines()
    recover_line = open(argv[3], "r").readlines()[0]
    recovers = recover_line.split(";")
    recover_dic = {}
    str_list = []
    if len(recovers) != 0:
        for recover in recovers:
            print(recover)
            tokens = recover.split("->")
            if len(tokens) == 2:
                if tokens[1] != "str":
                    recover_dic[tokens[1]] = tokens[0]
                else:
                    str_list.append(tokens[0])
    predictions_recover = []
    for predict in predictions:
        tokens = predict.split(" ")
        recover_line = ""
        str_num = 1
        for token in tokens:
            if token in recover_dic.keys():
                token = recover_dic[token]
            elif token == "str":
                if str_num <= len(str_list):
                    token = str_list[str_num-1]
                    str_num += 1
                else:
                    str_num += 1
            recover_line += token
        predictions_recover.append(recover_line)
    white_space_before_buggy_line = buggy_line[0:buggy_line.find(buggy_line.lstrip())]
    for i in range(len(predictions_recover)):
        output_file = os.path.join(argv[4], str(i+1), os.path.basename(argv[0]))
        os.makedirs(os.path.dirname(output_file))
        output_file = open(output_file, "w")
        for j in range(len(buggy_file_lines)):
            if(j+1 == buggy_line_number):
                last_char = buggy_file_lines[j].rstrip()[-1]
                if predictions_recover[i][-1] != last_char:
                    output_file.write(white_space_before_buggy_line + predictions_recover[i].rstrip()+last_char+"\n")
                else:
                    output_file.write(white_space_before_buggy_line + predictions_recover[i])
            else:
                output_file.write(buggy_file_lines[j])
        output_file.close()


if __name__=="__main__":
    argv = []
    argv.append("D:\\workspace\\eclipse2018\\Migration\\test\\AccountInstance.java")
    argv.append("15")
    argv.append("D:\\workspace\\eclipse2018\\Migration\\test\\abstract.txt")
    argv.append("D:\\workspace\\eclipse2018\\Migration\\test\\recover.txt")
    argv.append("D:\\workspace\\eclipse2018\\Migration\\test")
    main(sys.argv)
