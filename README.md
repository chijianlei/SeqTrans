# SeqTrans

A framework for automatic vulnerability fixing based on neural machine translation.  

## Description

SeqTrans is a tool that automatically learns the fix model for vulnerability fixing based on machine learning methods. It has the following features:
* We use the Neural Machine Translation (NMT) model transformer to learn and generalize common patterns from historical data for vulnerability fixing.  
* We propose to leverage data-flow dependencies to construct vulnerable sequences and maintain the vital context around them.
* Fine-tuning has been introduced to overcome the small sample size problem.
* Our SeqTrans outperforms other program repair techniques and achieves the accuracy of 23.3% in statement-level validation and 25.3% in CVE-level validation.

## Prerequisite

SeqTrans works fine with Java 1.8 and Pyhon 3.7.  
Gumtree and Understand (Version build 996 or below) are needed to extract fine-grained code diffs from history fixing records.   
We have made extensive changes to Gumtree's code, please use our modified version.  
To parse the source code and generate the abstract syntax tree, SrcML (Version 1.0.0) is used as the syntax parser.  
OpenNMT (Version 2.0.0 or above) is required to train the NMT model.  
Other third-party dependencies can be found in requirements.txt and pom.xml. 

## Dataset
SeqTrans uses two datasets to train the NMT model.  
One bug fix dataset was used to generate the pre-trained model, which contains 2 million fix records.  
The dataset can be found in here: https://sites.google.com/view/learning-fixes.  
One vulnerability fix dataset was used to fine-tune the model, which contains 1300 CVE fix records.  
The dataset can be found in here: https://github.com/ZeoVan/vulnerability-assessment-kb.  

## Experimental Results
Experimental results and the trained model are too large, we have shared them in Google Driver:  
https://drive.google.com/drive/folders/1gzUVyKoEFtK55hZcBlNL004oB9El6RzI?usp=sharing  

## Documentation

Documentation will be released soon.

## Supported languages

Currently only Java language is supported, more languages are coming soon.

## Citing GumTree

If you use SeqTrans in an academic work we would be really happy. 
This work is currently being submitted to TSE. If you cite our seminal paper, please using the following bibtex:

```
@article{chi2020seqtrans,
  title={SeqTrans: Automatic Vulnerability Fix via Sequence to Sequence Learning},
  author={Chi, Jianlei and Qu, Yu and Liu, Ting and Zheng, Qinghua and Yin, Heng},
  journal={arXiv preprint arXiv:2010.10805},
  year={2020}
}
```