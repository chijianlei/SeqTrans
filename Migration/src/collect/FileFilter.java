package collect;

import gumtreediff.gen.srcml.SrcmlJavaTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.TreeContext;
import structure.API;
import structure.Migration;
import utils.FileOperation;

import java.io.*;
import java.util.*;

public class FileFilter {
	private static LinkedHashSet<API> apis = new LinkedHashSet<API>();
	
	public static void main(String[] args) throws Exception{	
		String path = "apis";
//		apis = ReadAPI.readAPI(path);
//		for(String api : apis) {
//			System.out.println(api);
//		}
//		FileFilter.Filter("J:\\test1\\");
	}
	
	public static void Filter(String path) throws Exception {
		File validFile = new File("validFiles.txt");
		BufferedWriter wr = new BufferedWriter(new FileWriter(validFile));
		File rootDir = new File(path);
		File[] dirs = rootDir.listFiles();
		HashMap<String, HashSet<String>> importMap = getImportList();
		HashSet<String> imports = new HashSet<String>();
		for(Map.Entry<String, HashSet<String>> entry : importMap.entrySet()) {
			String key = entry.getKey();
			imports.add(key);
		}
		for(File dir : dirs) {
			System.out.println("Analyse "+dir.getName());
			if(dir.listFiles().length!=3) {//two commit dirs and the diff logs
				wr.close();
				throw new Exception("Error!");
			}
			String diffPath = dir.getAbsolutePath()+"\\diffs.txt";
			File diffFile = new File(diffPath);
			BufferedReader br = new BufferedReader(new FileReader(diffFile));
			String tmpline = br.readLine();
			String oldCommit = tmpline.split(";")[0];
			String newCommit = tmpline.split(";")[1];
			while((tmpline=br.readLine())!=null) {
				String[] diff = tmpline.split(";");
				String srcDiff = diff[0];
				String tgtDiff = diff[1];
				String srcPath = dir.getAbsolutePath()+"\\"+oldCommit+"\\"+srcDiff;
				String tgtPath = dir.getAbsolutePath()+"\\"+newCommit+"\\"+tgtDiff;
				BufferedReader br1 = new BufferedReader(new FileReader(new File(srcPath)));
				BufferedReader br2 = new BufferedReader(new FileReader(new File(tgtPath)));
				String tmpline1 = "";
				Boolean preserve = false;
				Boolean containsImport = false;
				ArrayList<String> tmpImports = new ArrayList<String>();
				while((tmpline1=br1.readLine())!=null) {//��srcFile��������API�ļ�
					if(tmpline1.contains("import")) {
						if(tmpline1.split(" ").length<2)
							continue;
						String className = tmpline1.split(" ")[tmpline1.split(" ").length-1];
						className = className.substring(0, className.length()-1);//delete ";"
						if(imports.contains(className)) {
							containsImport = true;
							tmpImports.add(className);
						}
					}
					if(containsImport==true) {
						for(String imp : tmpImports) {
							HashSet<String> methods = importMap.get(imp);
							for(String tmp : methods) {
								if(tmpline1.contains(tmp)) {
									preserve = true;
									break;
								}
							}
						}
					}
				}
				br1.close();
				containsImport = false;//reset
				tmpImports = new ArrayList<String>();//reset
				while((tmpline1=br2.readLine())!=null) {//��tgtFile��������API�ļ�
					if(tmpline1.contains("import")) {
						if(tmpline1.split(" ").length<2)
							continue;
						String className = tmpline1.split(" ")[tmpline1.split(" ").length-1];
						className = className.substring(0, className.length()-1);//delete ";"
						if(imports.contains(className)) {
							containsImport = true;
							tmpImports.add(className);
						}
					}
					if(containsImport==true) {
						for(String imp : tmpImports) {
							HashSet<String> methods = importMap.get(imp);
							for(String tmp : methods) {
								if(tmpline1.contains(tmp)) {
									preserve = true;
									break;
								}
							}
						}
					}
				}
				br2.close();
				if(preserve==true) {
					wr.append(srcPath+";"+tgtPath);
					wr.newLine();
					wr.flush();
				}
			}
			br.close();
		}
		wr.close();
	}	

	private static HashMap<String, HashSet<String>> getImportList() throws Exception {
		if(apis.size()==0)
			throw new Exception("error!");
		HashMap<String, HashSet<String>> importMap = new HashMap<String, HashSet<String>>();
		for(API api : apis) {
			String methodName = api.getMethodName();
			String importName = api.getLongName().substring(0, api.getLongName().lastIndexOf("."));
			if(!importMap.containsKey(importName)) {
				HashSet<String> list = new HashSet<String>();
				list.add(methodName);
				importMap.put(importName, list);
			}else {
				importMap.get(importName).add(methodName);
			}
		}
		return importMap;
	}
	
	public static ArrayList<Migration> readMigrationList(String path, String filter) throws Exception{
		ArrayList<Migration> migrates = new ArrayList<Migration>();
		File cpFile = new File(path);
		System.err.println("Analyse:"+ cpFile.getName());
		String diffPath = cpFile.getAbsolutePath()+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if(!diffFile.exists())
			throw new Exception("file is not existed!");
		BufferedReader br = new BufferedReader(new FileReader(diffFile));
		String tmpline = br.readLine();
		String repoName = tmpline;
		tmpline = br.readLine();
		String srcHash = tmpline.split(";")[0];
		String dstHash = tmpline.split(";")[1];
		
		while((tmpline=br.readLine())!=null) {
			String path1 = tmpline.split(";")[0];
			String path2 = tmpline.split(";")[1];
			path1 = cpFile.getPath()+"//"+srcHash+"//"+path1;
			path2 = cpFile.getPath()+"//"+dstHash+"//"+path2;
			File srcFile = new File(path1);
			if (!srcFile.exists()) {
				br.close();
				throw new Exception("srcfile is not existed!");
			}
			System.out.println("Analyse:"+ srcFile.getName());
			Calendar calendar = Calendar.getInstance();
			Date time = calendar.getTime();
			System.out.println(time);
			File dstFile = new File(path2);	
			if (!dstFile.exists()) {
				br.close();
				throw new Exception("dstfile is not existed!");
			}
			try {
				TreeContext tc1 = new SrcmlJavaTreeGenerator().generateFromFile(srcFile);
				//need to be changed by different languages
				TreeContext tc2 = new SrcmlJavaTreeGenerator().generateFromFile(dstFile);
				Matcher m = Matchers.getInstance().getMatcher(tc1.getRoot(), tc2.getRoot());
		        m.match();
		        MappingStore mappings = m.getMappings();
				System.out.println("Mapping size: "+mappings.asSet().size());
				Migration mi = new Migration(tc1, tc2, mappings, srcFile.getAbsolutePath(), dstFile.getAbsolutePath());
				mi.setRepoName(repoName);
				mi.setSrcHash(srcHash);
				mi.setDstHash(dstHash);
				migrates.add(mi);
			} catch (Exception e) {
				continue;
				// TODO: handle exception
			}												
		}	
		br.close();
		return migrates;
	}
	
	public static ArrayList<Migration> readTufanoList(String path) throws Exception{
		ArrayList<Migration> migrates = new ArrayList<Migration>();
		File cpFile = new File(path);
		String hashID = cpFile.getName();
		System.err.println("Analyse:"+ cpFile.getName());
		String srcDiffPath = cpFile.getAbsolutePath()+"\\P_dir\\";
		String dstDiffPath = cpFile.getAbsolutePath()+"\\F_dir\\";
		ArrayList<File> srcList = new ArrayList<File>();		
		ArrayList<File> dstList = new ArrayList<File>();
		FileOperation.traverseFolder(srcDiffPath, srcList);
		FileOperation.traverseFolder(dstDiffPath, dstList);
		if(srcList.size()==0||dstList.size()==0) {
			System.err.println("file is not existed!");
			return migrates;
		}		
	    System.out.println("FileSize:"+srcList.size()+","+dstList.size());
		if(srcList.size()>dstList.size()) {
			System.err.println("file number is not the same!");
			return migrates;
		}
		
		for(File srcFile : srcList) {
			if(srcFile.length()>1048000)
				continue;// skip the file that bigger than 2MB
		    if(srcFile.getName().equals("Run.java")||srcFile.getName().equals("DatabaseVersioningService.java"))
		    	continue;//This file takes too much time to analyze
			String beforeName = srcFile.getName();
			int count = 0;
			File targetFile = null;
			List<File> targetList = new ArrayList<File>();
			for(File dstFile : dstList) {
				String afterName = dstFile.getName();
				if(afterName.equals(beforeName)) {
					count++;
					targetList.add(dstFile);
				}
			}
			if(count==0) {
				throw new Exception("dstfile is not existed!");
			}else if(count ==1) {
				targetFile = targetList.get(0);
			}else if(count>1){
				String targetPath = srcFile.getAbsolutePath();
				String[] targetPaths = targetPath.split("\\\\");
				targetPath = targetPath.replace("P_dir", "F_dir");
				Boolean find = false;
				for(File tmpFile : targetList) {
					if(tmpFile.getAbsolutePath().equals(targetPath)) {
						targetFile = tmpFile;
						find = true;
					}
				}
					
				if(find==false) {//find the file that has the most similar score
					int score = 0;
					System.err.println("contain the duplicate!");
					for(File tmpFile : targetList) {
						String[] tmpPaths = tmpFile.getAbsolutePath().split("\\\\");
						int tmpScore = 0;
						for(int i=0;i<tmpPaths.length-1&&i<targetPaths.length-1;i++) {
							if(tmpPaths[i].equals(targetPaths[i]))
								tmpScore++;
						}
						if(tmpScore>score) {
							score = tmpScore;
							targetFile = tmpFile;
							find = true;
						}else if(score!=0&&tmpScore==score) {
							System.err.println(targetFile.getAbsolutePath());
							throw new Exception("Duplicate dst file!");
						}							
					}
				}
			}
			System.out.println("Analyse:"+ beforeName+", Size:"+srcFile.length());
			
			Calendar calendar = Calendar.getInstance();
			Date time = calendar.getTime();
			System.out.println(time);
			try {
				TreeContext tc1 = new SrcmlJavaTreeGenerator().generateFromFile(srcFile);
				TreeContext tc2 = new SrcmlJavaTreeGenerator().generateFromFile(targetFile);
				Matcher m = Matchers.getInstance().getMatcher(tc1.getRoot(), tc2.getRoot());
		        m.match();
		        MappingStore mappings = m.getMappings();
				Migration mi = new Migration(tc1, tc2, mappings, srcFile.getAbsolutePath(), targetFile.getAbsolutePath());
				mi.setRepoName(hashID);
				System.out.println("Mapping size: "+mappings.asSet().size());
				migrates.add(mi);
			} catch (Exception e) {
				continue;
				// TODO: handle exception
			}	
		}			
		return migrates;
	}	
	
	
}
