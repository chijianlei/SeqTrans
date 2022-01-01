package utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class FilterFiles {

	public static void main (String args[]) throws Exception{
		String dirPath1 = "tmp\\ROS1_src\\";
		String dirPath2 = "tmp\\ROS2_src\\";
		String outPath = "tmp\\output\\";
		ArrayList<File> fileList1 = new ArrayList<>();
		traverseFolder(dirPath1, fileList1);
		System.out.println("size:"+fileList1.size());
		ArrayList<File> fileList2 = new ArrayList<>();
		traverseFolder(dirPath2, fileList2);
		FileOperation fo = new FileOperation();
		for(File file1 : fileList1) {
			String fileName1 = file1.getName();
			Boolean ifFind = false;
			for(File file2 : fileList2) {
				String fileName2 = file2.getName();
				if(fileName1.equals(fileName2)) {
					String data1 = ReadFileToString(file1);
					String data2 = ReadFileToString(file2);
					if(data1.equals(data2)) {
						System.out.println(fileName1+" is identitcal");
						ifFind = true;
					}else {
						String javaName1 = fileName1.substring(0, fileName1.length()-4);
						String javaName2 = fileName2.substring(0, fileName2.length()-4);
						String newFilePath1 = outPath+javaName1+"\\"+javaName1+"1.cpp";
						String newFilePath2 = outPath+javaName2+"\\"+javaName2+"2.cpp";
						File newFile1 = new File(newFilePath1);
						File newFile2 = new File(newFilePath2);
						fo.copyFile(file1, newFile1);
						fo.copyFile(file2, newFile2);
						ifFind = true;
						break;
					}
				}
			}
			if(!ifFind) {
				System.out.println("Cannot find "+fileName1);
			}
		}

	}

	public static String ReadFileToString(File file) throws IOException {
		StringBuilder fileData = new StringBuilder(1000);
		BufferedReader reader = new BufferedReader(new FileReader(file));

		char[] buf = new char[10];
		int numRead = 0;
		while ((numRead = reader.read(buf)) != -1) {
			String readData = String.valueOf(buf, 0, numRead);
			fileData.append(readData);
			buf = new char[1024];
		}
		reader.close();
		return  fileData.toString();
	}


    public static void traverseFolder(String path, ArrayList<File> fileList) {
		File dir = new File(path);
		if (dir.exists()) {
			File[] files = dir.listFiles();
			if (files.length == 0) {
				System.out.println("文件夹是空的!");
			} else {
				for (File file : files) {
					if (file.isDirectory()) {
						traverseFolder(file.getAbsolutePath(), fileList);
					} else {
						if(file.getName().contains(".cpp")||file.getName().contains(".CPP"))
						fileList.add(file);
					}
				}
			}
		} else {
			System.out.println("文件不存在!");
		}
	}

}
