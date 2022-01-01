import understand
import json

db = understand.open('D:\\workspace\\android-27.udb')


def sortKeyFunc(ent):
    return str.lower(ent.longname())


def printKindType(ents):
    kinds = list()
    unknowList = list()
    nameDict = {}
    for func in sorted(ents, key=sortKeyFunc):
        kind = func.kindname()
        if kind not in kinds:
            kinds.append(kind)
            nameDict[kind] = 1;
            if 'Unknown'in str(kind):
                unknowList.append(func.longname())
        else:
            nameDict[kind] = nameDict[kind]+1
            if 'Implicit'in str(kind):
                unknowList.append(func.longname())
    for each in unknowList:
        print(each)
    for key, value in nameDict.items():
        print(key, value)


filename = 'api.txt'
wr = open(filename, 'w')

ents = db.ents("function,method,procedure")
apis = list()
printKindType(ents)
for func in sorted(ents, key=sortKeyFunc):
    api = {}
    kind = str(func.kindname())
    names = str(func.longname()).split(".")
    api['Name'] = str(func.longname())
    ifPrivate = False
    # if names[0] != 'torch':
    #     continue
    # for eachName in names:
    #     if eachName[0:1] == '_':
    #         ifPrivate = True
    #         break
    # if names[len(names)-1][0:1] == '_':
    #     ifPrivate = True
    if 'Public' not in kind:
        ifPrivate = True
    if 'Abstract' in kind:
        ifPrivate = True
    if 'Implicit' in kind:
        ifPrivate = True
    if names[0] != 'android':
        ifPrivate = True
    if 'Test' in func.longname():
        ifPrivate = True
    if ifPrivate is False:
        # print(func.longname(), " (", sep="", end="")
        wr.write(func.longname() + ";")
        first = True
        params = list()
        for param in func.ents("Define", "Parameter"):
            if not first:
                # print(", ", end="")
                wr.write(",")
            # print(param.type(), param, end="")
            wr.write(str(param.type()))
            params.append(str(param.type())+' '+str(param))
            first = False
        # print(")")
        wr.write('\n')
        api['Param'] = params
        if api not in apis:
            apis.append(api)

# for api in apis:
#     wr.write(api['Name']+' (')
#     params = api['Param']
#     first = True
#     for par in params:
#         if not first:
#             wr.write(', ')
#         wr.write(par)
#         first = False
#     wr.write(')\n')



