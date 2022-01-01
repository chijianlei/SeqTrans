package test;

import java.io.File;
import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecuteResultHandler;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteException;
import org.eclipse.jgit.api.Git;
import org.eclipse.jgit.lib.Ref;
import org.eclipse.jgit.lib.Repository;
import org.eclipse.jgit.storage.file.FileRepositoryBuilder;

public class test {
	public static void main(String[] args) throws Exception{	
		String classPath = "J:\\git_repo\\okhttp";
		FileRepositoryBuilder builder = new FileRepositoryBuilder();
		builder.setMustExist(true);
		builder.addCeilingDirectory(new File(classPath));
		builder.findGitDir(new File(classPath));
		Repository repo;
		repo = builder.build();
		Map<String, Ref> maps = repo.getAllRefs();
		for(Map.Entry<String, Ref> entry : maps.entrySet()) {
			String key = entry.getKey();
			System.out.println(repo.shortenRefName(key));
		}
//		Set<String> refs = repo.getRemoteNames();
//		for(String name : refs) {
//			System.out.println(name);
//		}
	}

	private void test1() throws Exception, IOException {
		System.out.println("RunExec");
		String versionCommit="78351302b0761178581d92612b528f6eea529618";//需要分析的Commit Hash		
		String path="D:\\workspace\\eclipse2018\\Migration\\OpenNMT-py\\";//对应项目在本地Repo的路径
		String line = "cmd.exe /C git checkout "+versionCommit;
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecuteResultHandler resultHandler = new DefaultExecuteResultHandler();
		DefaultExecutor executor = new DefaultExecutor();	
		executor.setWorkingDirectory(new File(path));
		executor.setExitValue(1);	//设置命令执行退出值为1，如果命令成功执行并且没有错误，则返回1		 
		executor.execute(cmdLine, resultHandler);
		resultHandler.waitFor();
	}
}
