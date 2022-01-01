package utils;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

public class FileOperation {

	 public void copyFile(File sourcefile,File targetFile) throws IOException{
		 if(!targetFile.getParentFile().exists())
			 targetFile.getParentFile().mkdirs();
	     //新建文件输入流并对它进行缓冲
		 FileInputStream input=new FileInputStream(sourcefile);
		 BufferedInputStream inbuff=new BufferedInputStream(input);
	     //新建文件输出流并对它进行缓冲
	     FileOutputStream out=new FileOutputStream(targetFile);
	     BufferedOutputStream outbuff=new BufferedOutputStream(out);
	     //缓冲数组
	     byte[] b=new byte[1024*5];
	     int len=0;
	     while((len=inbuff.read(b))!=-1){
	    	 outbuff.write(b, 0, len);
	     }

	     outbuff.flush(); //刷新此缓冲的输出流
	     inbuff.close();//关闭流
	     outbuff.close();
	     out.close();
	     input.close();
	 }

	 public void copyDirectiory(String sourceDir,String targetDir) throws IOException{
		 //新建目标目录
		 File source = new File(sourceDir);
		 File target = new File(targetDir);
		 if(!target.exists()){
			 target.mkdirs();
		 }
		 //获取源文件夹当下的文件或目录
		 File[] file=source.listFiles();
		 for (File sourceFile : file) {
			 if(sourceFile.isFile()){
				 //目标文件
				 File targetFile=new File(new File(targetDir).getAbsolutePath()+File.separator+sourceFile.getName());
				 copyFile(sourceFile, targetFile);
			 }
			 if(sourceFile.isDirectory()){
				 //准备复制的源文件夹
				 String dir1=sourceDir+"\\"+sourceFile.getName();
				 //准备复制的目标文件夹
				 String dir2=targetDir+"\\"+sourceFile.getName();
				 copyDirectiory(dir1, dir2);
			 }
		 }
	 }

	 public void delFolder(String folderPath) {
	     try {
	        delAllFile(folderPath); //删除完里面所有内容
	        String filePath = folderPath;
	        filePath = filePath.toString();
	        java.io.File myFilePath = new java.io.File(filePath);
	        myFilePath.delete(); //删除空文件夹
	     } catch (Exception e) {
	       e.printStackTrace();
	     }
	}//删除指定文件夹下所有文件param path 文件夹完整绝对路径

	 public boolean delAllFile(String path) {
	       boolean flag = false;
	       File file = new File(path);
	       if (!file.exists() || !file.isDirectory()) {
	         return flag;
	       }
	       String[] tempList = file.list();
	       File temp = null;
	       for (String element : tempList) {
	          if (path.endsWith(File.separator)) {
	             temp = new File(path + element);
	          } else {
	              temp = new File(path + File.separator + element);
	          }
	          if (temp.isFile()) {
	             temp.delete();
	          }
	          if (temp.isDirectory()) {
	             delAllFile(path + "/" + element);//先删除文件夹里面的文件
	             delFolder(path + "/" + element);//再删除空文件夹
	             flag = true;
	          }
	       }
	       return flag;
	     }
}
