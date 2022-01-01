package cost_matrix;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

public class Test {

	private static HashMap<Integer, String> files = new HashMap<>();
	private static double[][] Smatrix;

	/**
	 * 比较包内外的相似度
	 */

	public static void main(String[] args) throws Exception {
		Test test = new Test();
		String path1 = "file.txt";
		String path2 = "test.txt";
		String path3 = "target.txt";
		ArrayList<String> targetfile = new ArrayList<>();
		test.ReadFile(path1, path2);
		File file = new File(path3);
		BufferedReader br = new BufferedReader(new FileReader(file));
		String line = "";
		while((line=br.readLine())!=null){
			String tmp = line;
			targetfile.add(tmp);
		}
		br.close();
//		for(int i=0;i<targetfile.size();i++){
//			String tmp = targetfile.get(i);
//			double avg = test.avgS(tmp);//计算相同文件夹相似度
//			System.out.println("avg:"+avg);
//			System.out.println("--------------");
//		}
		double[][] matrix = new double[targetfile.size()][targetfile.size()];
		for(int i=0;i<targetfile.size();i++){
			String name1 = targetfile.get(i);
			for(int j=0;j<targetfile.size();j++){
				String name2 = targetfile.get(j);
				if(name1.equals(name2))
					continue;
				double avg = test.avgS(name1, name2);//计算不同文件夹间相似度
				System.out.println("name1:"+name1+" name2:"+name2);
				System.out.println("avg:"+avg);
				System.out.println("--------------");
				matrix[i][j] = avg;
			}
		}

		String path4 = "matrix.csv";
		File file2 = new File(path4);
		BufferedWriter wr = new BufferedWriter(new FileWriter(file2));
		for (String element : targetfile) {
			wr.append(element+",");
		}
		wr.newLine();
		wr.flush();
		for(int i=0;i<targetfile.size();i++){
			for(int j=0;j<targetfile.size();j++){
				if(i==j){
					wr.append("1,");
					continue;
				}
				wr.append(String.valueOf(matrix[i][j])+",");
			}
			wr.newLine();
			wr.flush();
		}//输出包间相似度矩阵
		wr.close();

		String path5 = "matrix1.csv";
		File file3 = new File(path5);
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(file3));
		for(int i=0;i<targetfile.size();i++){
			double avg = 0.0;
			double max = 0.0;
			double min = 1.0;
			String maxname = "";
			for(int j=0;j<targetfile.size();j++){
				double tmp = matrix[i][j];
				if(i==j)
					continue;
				avg = avg+tmp;
				if(tmp>max){
					max = tmp;
					maxname = targetfile.get(j);
				}
				if(tmp<min)
					min = tmp;
			}
			avg = avg/(targetfile.size()-1);//平均包间相似度
			wr1.append(targetfile.get(i)+",");
			wr1.append(String.valueOf(avg)+",");
			wr1.append(String.valueOf(max)+",");
			wr1.append(maxname+",");
			wr1.append(String.valueOf(min)+",");
			wr1.newLine();
			wr1.flush();
		}
		wr1.close();
	}

	public void ReadFile(String path1, String path2) throws IOException{
		File file = new File(path1);
		BufferedReader br = new BufferedReader(new FileReader(file));
		String line = "";
		while((line=br.readLine())!=null){
			String[] lines = line.split(",");
			files.put(Integer.valueOf(lines[0]), lines[1]);
		}
		br.close();

		Smatrix = new double[files.size()][files.size()];
		File file1 = new File(path2);
		BufferedReader br1 = new BufferedReader(new FileReader(file1));
		String line1 = "";
		int num = 0;
		while((line1=br1.readLine())!=null){
			String[] lines = line1.split(",");
			for(int i=0;i<lines.length-1;i++){
				String tmp = lines[i];
				if(tmp.substring(0, 1).equals(" ")){
					double sm = Double.valueOf(tmp.substring(1, tmp.length()));
					Smatrix[num][i] = sm;
				}else {
					double sm = Double.valueOf(tmp);
					Smatrix[num][i] = sm;
				}
			}
			num++;
		}
		br1.close();
	}//读取相似度矩阵和文件名

	public double avgS(String name1) throws Exception{
		ArrayList<Double> avgs = new ArrayList<>();
		ArrayList<Integer> files1 = new ArrayList<>();
		for(Entry<Integer, String> entry : files.entrySet()){
			int key = entry.getKey();
			String value = entry.getValue();
			if(value.contains(name1)){
				files1.add(key);
			}
		}//矩阵中读入所需文件名对应信息
		System.out.println("size:"+files1.size());
		if(files1.isEmpty())
			throw new Exception("不能为空！");

		double avg = 0.0;
		for (Integer x : files1) {
			for (Integer y : files1) {
				if(Smatrix[x][y]==1.0)
					continue;
				avg = avg+Smatrix[x][y];
			}
		}
		avg = avg/(files1.size()*(files1.size()-1));
		return avg;
	}//计算同文件夹之间相似度

	public double avgS(String name1, String name2) throws Exception{
		ArrayList<Integer> files1 = new ArrayList<>();
		ArrayList<Integer> files2 = new ArrayList<>();
		ArrayList<Double> avgs = new ArrayList<>();
		for(Entry<Integer, String> entry : files.entrySet()){
			int key = entry.getKey();
			String value = entry.getValue();
			if(value.contains(name1)){
				files1.add(key);
			}
			if(value.contains(name2)){
				files2.add(key);
			}
		}//矩阵中读入所需文件名对应信息
		if(files1.isEmpty()||files2.isEmpty())
			throw new Exception("不能为空！");

		for(int i=0;i<files1.size();i++){
			double avg = 0.0;
			int x = files1.get(i);
			for (Integer y : files2) {
				avg= avg+Smatrix[x][y];
			}
			for (Integer y : files2) {
				avg= avg+Smatrix[y][x];
			}
			avg = avg/(2*files2.size());//此处不用-1
			System.out.println(avg);
			avgs.add(avg);
		}
		double avg = 0.0;
		for (Double tmp : avgs) {
			avg = avg+tmp;
		}
		avg = avg/files1.size();
		return avg;
	}//计算不同文件夹之间相似度

}
