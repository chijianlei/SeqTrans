package cost_matrix;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;


public class Normalization {
	private static double[][] Smatrix;//所有文件的相似度矩阵
	private static double Min;
	private static double Max;

	public static void getPointsList() throws IOException{
		String txtPath="D:\\workspace\\eclipse4.3\\Kieker_records_analysis-kieker1.9\\file.txt";
		BufferedReader br=new BufferedReader(new FileReader(txtPath));
		String str="";
		int size = 0;
		while((str=br.readLine())!=null && str!=""){
			size++;
		}
		br.close();
		System.out.println("size:"+size);//从file中读入矩阵大小
		Smatrix = new double[size][size];
		String txtPath1="D:\\workspace\\eclipse4.3\\Kieker_records_analysis-kieker1.9\\test.txt";
		BufferedReader br1=new BufferedReader(new FileReader(txtPath1));
		String str1="";
		int count=0;
		double tmp = 0;
		Min = 1.0;
		Max = 0;
		while((str1=br1.readLine())!=null && str1!=""){
			String[] temp = str1.split(",");
//			for(int i=0;i<temp.length-1;i++){
//				System.out.println(temp[i]);
//			}
			for(int i=0;i<temp.length-1;i++){
				if(i!=0&&!temp[i].isEmpty()){
					temp[i].substring(0,1);
					tmp = Double.parseDouble(temp[i]);
					if(tmp!=1.0){
						if(tmp<Min)
							Min = tmp;
						if(tmp>Max)
							Max = tmp;
					}
				}
				Smatrix[count][i] = Double.valueOf(temp[i]);
			}
			count++;
		}
		br1.close();
		System.out.println("Min:"+Min);
		System.out.println("Max:"+Max);//测试最大最小值
//		for(int i=0;i<Smatrix.length;i++){
//			for(int j=0;j<Smatrix.length;j++){
//				System.out.println(Smatrix[i][j]+",");
//			}
//		}//测试矩阵输入
		int num = 0;
		for(int i=0;i<size;i++){
			for(int j=0;j<size;j++){
				if(Smatrix[i][j]>0.3&&Smatrix[i][j]!=1){
					System.out.println(Smatrix[i][j]+","+i+","+j);
					num++;
				}
			}
		}
		System.out.println("num:"+num);

//		String outpath = "D:\\workspace\\eclipse4.3\\Kieker_records_analysis-kieker1.9\\test1.txt";
//		double x,x1 =0;
//		for(int i=0;i<size;i++){
//			for(int j=0;j<size;j++){
//				x = Smatrix[i][j];
//				x1 = (x-Min)/(Max-Min);
//				Writetxt(outpath,x1+", ");
//			}
//			Writetxt(outpath,"\r\n");
//		}//数值归一化

	}

	public static void Writetxt(String path,String Content) {
        FileWriter fw = null;
        try {
            fw = new FileWriter(path,true);
            fw.write(Content);
            fw.flush();
            fw.close();
        } catch (IOException e1) {
            e1.printStackTrace();
            System.out.println("写入失败");
            System.exit(-1);
        }
    }

	public static void main(String[] args) throws IOException {
		getPointsList();
	}

}
