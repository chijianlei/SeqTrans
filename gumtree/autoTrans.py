import os, sys
import gensim
import numpy as np
import joblib
from gensim import corpora, models



def run():
    defalutpath = '/home/chijianlei/OpenNMT-py/'
    os.chdir(defalutpath+'/tmp')
    os.system("java -jar "+defalutpath+"tmp/gumtree.jar "+sys.argv[1])
    #os.system("cp "+defalutpath+"tmp/src-test.txt "+defalutpath+"/data")
    os.system("rm log.txt")
    os.system("python "+defalutpath+"translate.py -model "+defalutpath+"model/ROS-model_step_100000.pt -src "+defalutpath+"tmp/src-test.txt -output "+defalutpath+"tmp/pred.txt -replace_unk -verbose -n_best 5 >> "+defalutpath+"tmp/log.txt")

run()
logFile = open('log.txt')
file = open('score.txt', 'w')

for line in logFile:
    if 'BEST HYP:' in line:
        count = 0
        while count < 5:
            line = logFile.readline()
            score = line.split(']')[0]
            score = score[2:score.__len__()]
            count = count+1
            file.write(score+'\n')

####################################################
def read_text_data(data_file):
    if 'txt' == data_file.split(".")[-1]:
        handle=open(data_file,'r')
        lines=handle.readlines()
        result_list=[]
        for each_line in lines:
            result_list.append(each_line)
    return result_list
####################################################

NUM_TOPICS=128
FILE_STRING="SourceCodeTxt.txt"
dictionary = corpora.Dictionary(line.lower().split() for line in open(FILE_STRING))
corpus = [dictionary.doc2bow(line.lower().split()) for line in open(FILE_STRING)]
tfidf_model = models.TfidfModel(corpus)
corpus_tfidf = tfidf_model[corpus]
lsi_model = models.LsiModel.load('lsi.model')
rf=joblib.load("rf.m")

####################################################
def change_impact_predict(this_line):

    this_corpus=dictionary.doc2bow(this_line.lower().split())
    this_corpus_tfidf=tfidf_model[this_corpus]
    this_lsi=lsi_model[this_corpus_tfidf]
    vec_list=[]
    for each_topic in this_lsi:
        vec_list.append(each_topic[1])
    data_test=np.array(vec_list)
    try:
        predprob = rf.predict(data_test.reshape(1, -1))[0]
        predprob_auc = rf.predict_proba(data_test.reshape(1, -1))[0][1]
    except:
        return 0, 0
    return predprob, predprob_auc
####################################################

fileName = "src-test.txt"
file = open(fileName, "r")
wr = open("impact.txt", "w")
lines = file.readlines()
for line in lines:
    predprob, predprobauc = change_impact_predict(line)
    wr.write(str(predprob)+","+str(predprobauc)+"\n")
