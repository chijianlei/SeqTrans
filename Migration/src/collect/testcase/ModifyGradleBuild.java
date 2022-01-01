package collect.testcase;

import java.io.BufferedReader;
import java.io.CharArrayWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;

public class ModifyGradleBuild {
	
	public static void main(String[] args) throws Exception {
		String cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full";
		searchURL(cpRoot);
	}

	public static void searchURL(String cpRoot) throws Exception {
		File[] cpList = (new File(cpRoot)).listFiles();
		ArrayList<String> noGradleList = new ArrayList<String>();
		for(File cpFile : cpList) {
			File[] cpDirs = cpFile.listFiles();
			for(File cpDir : cpDirs) {
				if(cpDir.isDirectory()) {
//					if(!cpFile.getName().equals("cp102"))
//						continue;
					System.out.println(cpDir.getName());
					Collection<File> collections = FileUtils.listFiles(cpDir, 
							FileFilterUtils.suffixFileFilter("build.gradle"), DirectoryFileFilter.INSTANCE);
					ArrayList<File> filters = new ArrayList<>(collections);
					Collection<File> collections1 = FileUtils.listFiles(cpDir, 
							FileFilterUtils.suffixFileFilter("build_properties.gradle"), DirectoryFileFilter.INSTANCE);
					ArrayList<File> filters1 = new ArrayList<>(collections1);
					if(filters.size()==0) {
						noGradleList.add(cpFile.getName());
					}
					System.out.println(cpFile.getName()+": "+filters.size());
					for(File gradleBuild : filters) {
						replaceTextContent(gradleBuild);
					}
					for(File gradleBuild : filters1) {
						replaceTextContent(gradleBuild);
					}
				}
			}
		}
		
		for(String no : noGradleList) {
			System.out.println(no);
		}
	}
	

     /**
	 * 替换文本文件中的字符串
	 * @param path
	 * @throws IOException
	 */
	public static void replaceTextContent(File replaceFile) throws IOException{
		    List<String> old_contents = FileUtils.readLines(replaceFile, "utf-8");
		    ArrayList<String> new_contents = new ArrayList<String>();
			//原有的内容
			String srcStr1 = "http?://repo.spring.io/plugins-release";        
			//要替换的内容
	        String replaceStr1 = "https://maven.aliyun.com/repository/spring-plugin";        
	        //原有的内容
			String srcStr3 = "http?://repo.springsource.org/plugins-release";        
			//要替换的内容
		    String replaceStr3 = "https://maven.aliyun.com/repository/spring-plugin";	
	        //原有的内容
			String srcStr4 = "http?://maven.google.com/";        
			//要替换的内容
	        String replaceStr4 = "https://maven.aliyun.com/repository/google";
	        //原有的内容
			String srcStr5 = "http?://plugins.gradle.org/m2/";        
			//要替换的内容
	        String replaceStr5 = "https://maven.aliyun.com/repository/gradle-plugin";
	        //原有的内容
			String srcStr6 = "http?://repo.grails.org/grails/core";        
			//要替换的内容
	        String replaceStr6 = "https://maven.aliyun.com/repository/grails-core";
	        //原有的内容
			String srcStr7 = "http?://repository.apache.org/snapshots/";        
			//要替换的内容
	        String replaceStr7 = "https://maven.aliyun.com/repository/apache-snapshots";
	        
	        String addString1 = "		maven {url 'https://maven.aliyun.com/repository/spring'}";
	        String addString2 = "		maven {url 'https://maven.aliyun.com/repository/public'}";
	        String addString3 = "		maven {url 'https://maven.aliyun.com/repository/jcenter'}";
	        String addString4 = "		maven {url 'https://maven.aliyun.com/repository/central'}";
	        for(int i=0; i<old_contents.size();i++) {
	        	String line = old_contents.get(i);
	        	line = line.replaceAll(srcStr1, replaceStr1); 
//	        	line = line.replaceAll(srcStr2, replaceStr2); 
	        	line = line.replaceAll(srcStr3, replaceStr3); 
	        	line = line.replaceAll(srcStr4, replaceStr4); 
	        	line = line.replaceAll(srcStr5, replaceStr5); 
	        	line = line.replaceAll(srcStr6, replaceStr6); 	        	
	        	line = line.replaceAll(srcStr7, replaceStr7); 	        	
//	        	System.out.println(line);
	        	new_contents.add(line);
	        	if(line.contains("repositories {")) {
	        		String nextLine = old_contents.get(i+1);
	        		if(!nextLine.contains("https://maven.aliyun.com/repository/spring")) {
			        	new_contents.add(addString1);
			        	new_contents.add(addString2);
			        	new_contents.add(addString3);
			        	new_contents.add(addString4);
	        		}
	        	}
	        }
	        FileUtils.writeLines(replaceFile, new_contents);
	}
}
