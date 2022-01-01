package collect.testcase;

import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.tuple.Pair;

import java.io.File;
import java.io.IOException;
import java.util.*;

/**
 * Count bug numbers for each repo and then sort them
 * @throws Exception 
 */

public class CountBugNum {
	private static HashMap<String, Integer> repoFreq = new HashMap<String, Integer>();
	private static ArrayList<String> fullHashs = new ArrayList<>();
	private static  HashMap<String, String> cveMap= new HashMap<>();
	private static  HashMap<String, String> cweMap= new HashMap<>();
	
	public static void main(String[] args) throws Exception {
		String rootPath = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full";
		CountBugNum cb = new CountBugNum();
		cb.countRepos(rootPath);
	}
	
	public List<Map.Entry<String, Integer>> countRepos(String rootPath) throws Exception {
		File rootFile = new File(rootPath);
		File[] fileList = rootFile.listFiles();
		ArrayList<Pair<String, String>> pairList = new ArrayList();
		for(int i=0;i<fileList.length;i++) {
			File cpFile = fileList[i];
			String cpPath = cpFile.getAbsolutePath();
			Pair<String, String> pair = searchRepo(cpPath);
			pairList.add(pair);
		}
		String pontaPath = "D:\\workspace\\Pycharm\\20191222-Vulnerability-dataset\\dataset.csv";
		readPontaCsv(pontaPath);
		String cwePath = "D:\\workspace\\Pycharm\\20191222-Vulnerability-dataset\\CVE2CWE-map.csv";
		readCWE(cwePath);

		HashMap<String, Integer> freqCWE = new HashMap<>();
		HashMap<String, ArrayList<String>> cweKinds = new HashMap<>();
		for(String hash1 : fullHashs){
			for(Pair<String, String> pair : pairList){
				String repoName = pair.getLeft();
				String hash2 = pair.getRight();
				if(hash2.contains(hash1)){
					String cve = cveMap.get(hash1);
					String cwe = cweMap.get(cve);
					System.out.println(cve);
					System.out.println(cwe);
					ArrayList<String> cweList = cweKinds.get(repoName);
					if(cweList==null||!cweList.contains(cwe)){
						if(freqCWE.containsKey(repoName)) {
							freqCWE.put(repoName, freqCWE.get(repoName)+1);
							if(cweList==null)
								cweList = new ArrayList<String>();
							cweList.add(cwe);
							cweKinds.put(repoName, cweList);
						}else {
							freqCWE.put(repoName, 1);
							if(cweList==null)
								cweList = new ArrayList<String>();
							cweList.add(cwe);
							cweKinds.put(repoName, cweList);
						}
					}
					break;
				}
			}
		}

		List<Map.Entry<String, Integer>> list = new ArrayList<Map.Entry<String, Integer>>(repoFreq.entrySet());
		Collections.sort(list, new Comparator<Map.Entry<String, Integer>>() {   
		    public int compare(Map.Entry<String, Integer> o1, Map.Entry<String, Integer> o2) {      
		        return (o2.getValue() - o1.getValue()); 
		        //return (o1.getKey()).toString().compareTo(o2.getKey());
		    }
		});
        
        for(Map.Entry<String, Integer> mapping: list){ 
               System.out.println(mapping.getKey()+":"+mapping.getValue()); 
        }
		System.out.println("-------------------------");

		List<Map.Entry<String, Integer>> list1 = new ArrayList<Map.Entry<String, Integer>>(freqCWE.entrySet());
		Collections.sort(list1, new Comparator<Map.Entry<String, Integer>>() {
			public int compare(Map.Entry<String, Integer> o1, Map.Entry<String, Integer> o2) {
				return (o2.getValue() - o1.getValue());
				//return (o1.getKey()).toString().compareTo(o2.getKey());
			}
		});

		for(Map.Entry<String, Integer> mapping: list1){
			System.out.println(mapping.getKey()+":"+mapping.getValue());
		}
        
        return list;
	}
	
	private static Pair<String, String> searchRepo(String cpPath) throws Exception {
		File cpFile = new File(cpPath);
//		System.out.println("Analyse:"+ cpFile.getName());
		String hash = null;
		String diffPath = cpFile.getAbsolutePath()+"\\diffs.txt";
		File diffFile = new File(diffPath);
		if(!diffFile.exists())
			throw new Exception("file is not existed!");
		List<String> lines = FileUtils.readLines(diffFile, "UTF-8");	
		String repoName = lines.get(0);
		hash = lines.get(1).split(";")[1];
		Pair<String, String> pair = Pair.of(repoName, hash);
		if(repoFreq.containsKey(repoName)) {
			repoFreq.put(repoName, repoFreq.get(repoName)+1);
		}else {
			repoFreq.put(repoName, 1);
		}
		return pair;
	}

	private static void readPontaCsv(String path) throws IOException {
		List<String> lines = FileUtils.readLines(new File(path), "utf-8");
		for(String line : lines){
			String[] tmps = line.split(",");
			String hash = tmps[2];
			String cve = tmps[0];
			fullHashs.add(hash);
			cveMap.put(hash, cve);
		}
	}

	private static void readCWE(String path) throws IOException {
		List<String> lines = FileUtils.readLines(new File(path), "utf-8");
		for(String line : lines){
			String[] tmps = line.split(",");
			String cve = tmps[0];
			String cwe = tmps[1];
			cweMap.put(cve, cwe);
		}
	}

	
}
