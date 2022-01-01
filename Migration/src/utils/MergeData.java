package utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;

public class MergeData {
	/**
	 * Merge training files into the final training set
	 * @throws Exception 
	 */
	
	public static void main(String[] args) throws Exception {
		String path = "jsons";
//		mergeTrainingSet(path);
		mergeJsonSet(path);
	}
	
	public static void mergeJsonSet(String dir) throws Exception {
		String path = "Json_merge\\";
		File outputDir = new File(path);
		if (!outputDir.exists()) {
			outputDir.mkdirs();
		}
		if(outputDir.listFiles().length!=0)
			throw new Exception("plz clean the dir!");
		File rootFile = new File(dir);
		File[] dirs = rootFile.listFiles();
		int count = 0;
		for(File cpDir : dirs) {
			System.out.println("处理文件夹: "+cpDir.getName());
			File[] files = cpDir.listFiles();
			if(files.length==0)
				continue;
			for(File cpFile : files) {
				String name = cpFile.getName();
				String newPath = path+"pair"+String.valueOf(count)+"_src.json";
				if(name.contains("src")) {
					String index = name.split("_")[0];					
					for(int i=0;i<files.length;i++) {
						File cpFile1 = files[i];
						String name1 = cpFile1.getName();
//						System.out.println(name1);
						if(name1.contains("tgt")&&name1.contains(index)) {						
							String newPath1 = path+"pair"+String.valueOf(count)+"_tgt.json";
							File targetFile = new File(newPath);
							File targetFile1 = new File(newPath1);
							FileOperation.copyFile(cpFile, targetFile);
							FileOperation.copyFile(cpFile1, targetFile1);
							count++;
							break;
						}else if(i==files.length-1)
							throw new Exception("error case, not find tgt json!");
					}
				}
			}
		}
	}
	
	public static void mergeTrainingSet(String dir) throws Exception {
		File dirFile = new File(dir);
		File[] files = dirFile.listFiles();
		ArrayList<String> defuses = new ArrayList<String>();
		ArrayList<String> srcs = new ArrayList<String>();
		ArrayList<String> tgts = new ArrayList<String>();
		for(File file : files) {
			String name = file.getName();
			BufferedReader br = new BufferedReader(new FileReader(file));
			String tmpline = "";
			ArrayList<String> lines = new ArrayList<String>();
			while((tmpline = br.readLine())!=null) {
				lines.add(tmpline);
			}
			if(name.contains("defuse")) {				
				for(String line : lines) 
					defuses.add(line);			
			}else if(name.contains("src-val")) {
				for(String line : lines)
					srcs.add(line);
			}else if (name.contains("tgt-val")) {
				for(String line : lines)
					tgts.add(line);					
			}
			br.close();
		}
		BufferedWriter wr = new BufferedWriter(new FileWriter(new File("defuse.txt")));
		BufferedWriter wr1 = new BufferedWriter(new FileWriter(new File("src-val.txt")));
		BufferedWriter wr2 = new BufferedWriter(new FileWriter(new File("tgt-val.txt")));
	    for(String line : defuses) {
	    	wr.append(line);
	    	wr.newLine();
	    	wr.flush();
	    }
	    for(String line : srcs) {
	    	wr1.append(line);
	    	wr1.newLine();
	    	wr1.flush();
	    }
	    for(String line : tgts) {
	    	wr2.append(line);
	    	wr2.newLine();
	    	wr2.flush();
	    }
	    wr.close();
	    wr1.close();
	    wr2.close();
	}

}
