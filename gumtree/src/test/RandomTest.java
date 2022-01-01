package test;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

import split.Split;
import structure.Migration;
import structure.SubTree;
import structure.Transform;
import utils.Output;

public class RandomTest {

	public static void main (String args[]) throws Exception{
		Split sp = new Split();
		ArrayList<Migration> migrats = new ArrayList<>();
		migrats = sp.readMigration("migrations", "");
		sp.storeTrans(migrats);
		ArrayList<Transform> trans = sp.trans;
		ArrayList<String> counts = new ArrayList<>();
		File file1 = new File("srcTest.txt");
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(file1));
		File file2 = new File("dstTest.txt");
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(file2));
		if(trans.size()<100) {
			System.err.println("error");
		}else {
			int[] randoms = randomNumber(0, trans.size()-1, 150);
			for (int random : randoms) {
				Transform tf = trans.get(random);
				SubTree sDT = tf.getSTree();
				SubTree dDT = tf.getDTree();
				if(sDT!=null&&dDT!=null) {
                    if(sDT.getTC()!=null&&dDT.getTC()!=null) {
                    	String srcLine = Output.subtree2src(sDT);
        				String dstLine = Output.subtree2src(dDT);
        	    		if(((float)srcLine.length()/(float)dstLine.length())<0.25||((float)srcLine.length()/(float)dstLine.length())<0.25) {
        	    			continue;
        	    		}//长度相差太多的句子直接跳过
        				wr1.append(srcLine);
        				wr1.newLine();
        				wr1.flush();
        				wr2.append(dstLine);
        				wr2.newLine();
        				wr2.flush();
        				counts.add(srcLine);
        				if (counts.size()==100) {
        					break;
        				}
	                }
				}
			}
		}
		wr1.close();
		wr2.close();
	}

	 /**
	    * 功能：产生min-max中的n个不重复的随机数
	    *
	    * min:产生随机数的其实位置
	    * mab：产生随机数的最大位置
	    * n: 所要产生多少个随机数
	    *
	    */
	    public static int[] randomNumber(int min,int max,int n){

	        //判断是否已经达到索要输出随机数的个数
	        if(n>(max-min+1) || max <min){
	            return null;
	        }

	        int[] result = new int[n]; //用于存放结果的数组

	        int count = 0;
	        while(count <n){
	            int num = (int)(Math.random()*(max-min))+min;
	            boolean flag = true;
	            for(int j=0;j<count;j++){
	                if(num == result[j]){
	                    flag = false;
	                    break;
	                }
	            }
	            if(flag){
	                result[count] = num;
	                count++;
	            }
	        }
	        return result;
	    }

}
