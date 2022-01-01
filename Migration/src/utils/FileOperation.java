package utils;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;

public class FileOperation {

	 public static void copyFile(File sourcefile,File targetFile) throws IOException{
		 File parent = targetFile.getParentFile();
		 if(!parent.exists()){
			parent.mkdirs();
		 }
		//新建文件输入流并对它进行缓冲
		 FileInputStream input=new FileInputStream(sourcefile);	        
		 BufferedInputStream inbuff=new BufferedInputStream(input);	        
		//新建文件输出流并对它进行缓冲
	     FileOutputStream out=new FileOutputStream(targetFile);
	     BufferedOutputStream outbuff=new BufferedOutputStream(out);  
	     ///缓冲数组   
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
	 
	 public static void copyDirectiory(String sourceDir,String targetDir) throws IOException{	        	        
		 //新建目标目录
		 File source = new File(sourceDir);
		 File target = new File(targetDir);
		 if(!target.exists()){
			 target.mkdirs();
		 }        	        
		//获取源文件夹当下的文件或目录	  	        
		 File[] file=source.listFiles();
		 for (int i = 0; i < file.length; i++) {	            
			 if(file[i].isFile()){	                
				//源文件	               
				 File sourceFile=file[i];	                   
				//目标文件	                
				 File targetFile=new File(new File(targetDir).getAbsolutePath()+File.separator+file[i].getName());	                	                
				 copyFile(sourceFile, targetFile);	            	           
			 }	            	            	            
			 if(file[i].isDirectory()){	               
				//准备复制的源文件夹               
				 String dir1=sourceDir+"\\"+file[i].getName();	               
				//准备复制的目标文件夹	               
				 String dir2=targetDir+"\\"+file[i].getName();	                	                
				 copyDirectiory(dir1, dir2);	            
			 }	        
		 }	        	    
	 }
	 
	 public static void delFolder(String folderPath) {
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
	 
	 public static boolean delAllFile(String path) {
	       boolean flag = false;
	       File file = new File(path);
	       if (!file.exists()) {
	         return flag;
	       }
	       if (!file.isDirectory()) {
	         return flag;
	       }
	       String[] tempList = file.list();
	       File temp = null;
	       for (int i = 0; i < tempList.length; i++) {
	          if (path.endsWith(File.separator)) {
	             temp = new File(path + tempList[i]);
	          } else {
	              temp = new File(path + File.separator + tempList[i]);
	          }
	          if (temp.isFile()) {
	             temp.delete();
	          }
	          if (temp.isDirectory()) {
	             delAllFile(path + "/" + tempList[i]);//先删除文件夹里面的文件
	             delFolder(path + "/" + tempList[i]);//再删除空文件夹
	             flag = true;
	          }
	       }
	       return flag;
	     }
	 
	public static void traverseFolder(String path, ArrayList<File> fileList) {
		File dir = new File(path);
		if (dir.exists()) {
			File[] files = dir.listFiles();
			if (files.length == 0) {
				System.out.println("error length!");
			} else {
				for (File file : files) {
					if (file.isDirectory()) {
						traverseFolder(file.getAbsolutePath(), fileList);
					} else {
						if(file.getName().contains(".java"))
						fileList.add(file);
					}
				}
			}
		} else {
			System.out.println("dir not exists");
		}		
	}
	 
}
