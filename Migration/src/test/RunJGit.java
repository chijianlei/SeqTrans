package test;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.eclipse.jgit.storage.file.FileRepositoryBuilder;
import org.eclipse.jgit.treewalk.CanonicalTreeParser;
import org.eclipse.jgit.api.Git;
import org.eclipse.jgit.api.errors.GitAPIException;
import org.eclipse.jgit.diff.DiffEntry;
import org.eclipse.jgit.diff.DiffEntry.ChangeType;
import org.eclipse.jgit.lib.ObjectId;
import org.eclipse.jgit.lib.ObjectReader;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.revwalk.RevCommit;
import org.eclipse.jgit.revwalk.RevWalk;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class RunJGit {
	
	static void printTime(int commitTime) {
		SimpleDateFormat formatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		String timestampString=String.valueOf(commitTime);
        Long timestamp = Long.parseLong(timestampString) * 1000;
        String date = formatter.format(new Date(timestamp));
        System.out.println(date);
	}
	
	static List<DiffEntry> getChangedFileList(RevCommit revCommit, Repository repo) {
		List<DiffEntry> returnDiffs = null;		
		try {
			RevCommit previsouCommit=getPrevHash(revCommit,repo);
			if(previsouCommit==null)
				return null;
			ObjectId head=revCommit.getTree().getId();
			
			ObjectId oldHead=previsouCommit.getTree().getId();
			
			System.out.println("Printing diff between the Revisions: " + revCommit.getName() + " and " + previsouCommit.getName());

            // prepare the two iterators to compute the diff between
    		try (ObjectReader reader = repo.newObjectReader()) {
        		CanonicalTreeParser oldTreeIter = new CanonicalTreeParser();
        		oldTreeIter.reset(reader, oldHead);
        		CanonicalTreeParser newTreeIter = new CanonicalTreeParser();
        		newTreeIter.reset(reader, head);

        		// finally get the list of changed files
        		try (Git git = new Git(repo)) {
                    List<DiffEntry> diffs= git.diff()
            		                    .setNewTree(newTreeIter)
            		                    .setOldTree(oldTreeIter)
            		                    .call();
                    for (DiffEntry entry : diffs) {
//                        System.out.println("Entry: " + entry);
                    }
                    returnDiffs=diffs;
        		} catch (GitAPIException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
    		}			
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return returnDiffs;
	}
	
	public static RevCommit getPrevHash(RevCommit commit, Repository repo)  throws  IOException {
	    try (RevWalk walk = new RevWalk(repo)) {
	        // Starting point
	        walk.markStart(commit);
	        int count = 0;
	        for (RevCommit rev : walk) {
	            // got the previous commit.
	            if (count == 1) {
	                return rev;
	            }
	            count++;
	        }
	        walk.dispose();
	    }
	    //Reached end and no previous commits.
	    return null;
	}

	public static void main(String[] args) throws IOException {
		// TODO Auto-generated method stub
//		String versionCommit="d0a64168441a08aa323a6881db444b935c186ee3";//ȷ���Ķ�Ӧ�汾��Commit hash��
//		String versionCommit="d1453fda74bff3d908218207229be1c72fe65166";//Xalan
//		String versionCommit="c79cb6f1a7bbdbba11dd130f54bb6b69a90c4777";//Abdera
//		String versionCommit="c13c4c115f6ceaa1a2fd8786ed638c30b0e4f505";//Camel-2.1.0
		String versionCommit="d1453fda74bff3d908218207229be1c72fe65166";//Xalan 2.4.0
		File perforFile = new File("Bug-Analysis-Results.csv");
	    FileWriter fw = new FileWriter(perforFile, true);
	    PrintWriter out = new PrintWriter(fw);
		
		ArrayList<Object[]> commitPairArray=new ArrayList<Object[]> (); 
		
//		String path="D:\\Projects\\Subject-Versions\\abdera";
//		String path="D:\\Projects\\Subject-Versions\\camel";
		String path="D:\\Projects\\Subject-Versions\\xalan-j";
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(path));
		builder.findGitDir(new File(path));
		JSONParser parser = new JSONParser();
		try {
			
			Object obj2 = parser.parse(new FileReader("Xalan-2.4-Severe/fix_and_introducers_pairs.json"));
			JSONArray jsonObject2 = (JSONArray) obj2;
			Iterator pairs = jsonObject2.iterator();
			while(pairs.hasNext()) {
				JSONArray eachPair=(JSONArray) pairs.next();
				Object[] eachArrayPair=eachPair.toArray();
				commitPairArray.add(eachArrayPair);
			}
			
			
			
			Repository repo=builder.build();
			RevWalk walk = new RevWalk(repo);
			
			ObjectId versionId=repo.resolve(versionCommit);
			RevCommit verCommit=walk.parseCommit(versionId);
			int verCommitTime=verCommit.getCommitTime();
			System.out.println("The sepecific version's commit time: ");
			printTime(verCommitTime);
			
//			String issuePath="issue_list.json";
//			String input = FileUtils.readFileToString(new File(issuePath), "UTF-8");
			Object obj = parser.parse(new FileReader("Xalan-2.4-Severe/issue_list.json"));
			JSONObject jsonObject = (JSONObject) obj;
			Set issues=jsonObject.keySet();
			for(Object issue:issues) {
				JSONObject content=(JSONObject) jsonObject.get(issue);
				String commitHash=(String) content.get("hash");
				ObjectId objId = repo.resolve(commitHash);
				
				RevWalk walk2 = new RevWalk(repo);
		        RevCommit revCommit = walk2.parseCommit(objId);		        
		        
		        ArrayList<String> fixFiles=new ArrayList<String>();
		        
		        ArrayList<String> thisIssueFile=new ArrayList<String>();
		        System.out.println("The issue: "+issue);
		        System.out.println("Analyzing the commit: "+commitHash);
		        int fixTime=revCommit.getCommitTime();
		        if(verCommitTime>fixTime)
		        	continue;
		        List<DiffEntry> diffFix=getChangedFileList(revCommit,repo);
		        if(diffFix==null)
		        	continue;
		        for (DiffEntry entry : diffFix) {
		        	fixFiles.add(entry.getNewPath());
//                    if(entry.getChangeType().equals(ChangeType.MODIFY)) {
//                    	System.out.println(entry.getNewPath());
//                    	fixFiles.add(entry.getNewPath());
//                    }
                }
		        
		        for (int i = 0; i < commitPairArray.size(); i++) {
		        	Object[] eachArrayPair=commitPairArray.get(i);
		        	String fixCommit=(String)eachArrayPair[0];
		        	if(fixCommit.equals(commitHash)) {
//		        		System.out.println(fixCommit+","+(String)eachArrayPair[1]);
		        		String introHash=(String)eachArrayPair[1];
//		        		String introHash="ecf89a60bfd8089d1b1de5666bd2e9d5938abe8e";
		        		ObjectId introId = repo.resolve(introHash);
		        		
		        		RevWalk walk3 = new RevWalk(repo);
				        RevCommit introCommit = walk3.parseCommit(introId);
				        int introTime=introCommit.getCommitTime();
				        if(introTime<=verCommitTime && verCommitTime<=fixTime) {
				        	System.out.println("Find!!!");
				        	System.out.println("The fix's time: ");
					        printTime(fixTime);
					        System.out.println(commitHash);
					        System.out.println(fixFiles);
					        System.out.println("------------------");
					        
				        	System.out.println("The introduce time: ");
				        	printTime(introTime);
				        	System.out.println(introHash);
				        	ArrayList<String> introFiles=new ArrayList<String>();
				        	List<DiffEntry> diffIntro=getChangedFileList(introCommit,repo);
				        	if(diffIntro==null)
					        	continue;
				        	for (DiffEntry entry : diffIntro) {
				        		introFiles.add(entry.getNewPath());
//			                    if(entry.getChangeType().equals(ChangeType.MODIFY)||entry.getChangeType().equals(ChangeType.ADD)) {
//			                    	System.out.println(entry.getNewPath());
//			                    	introFiles.add(entry.getNewPath());
//			                    }
			                }
				        	System.out.println(introFiles);
				        	System.out.println("------------------");
				        	introFiles.retainAll(fixFiles);
				        	System.out.println("The remained list:");
				        	System.out.println(introFiles);
				        	if(introFiles.size()!=0)
				        		for(String eachFile:introFiles) {
				        			if(!thisIssueFile.contains(eachFile))
				        				thisIssueFile.add(eachFile);
				        		}
				        	System.out.println("------------------");
				        	
				        }				        			        	
		        	}       		
		        }
		        if(thisIssueFile.size()!=0) {
			        out.write(issue+","+String.join(",", thisIssueFile)+"\n");
			        out.flush();
		        }
			}
			
		} catch (IOException | ParseException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}

