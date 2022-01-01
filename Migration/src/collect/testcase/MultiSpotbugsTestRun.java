package collect.testcase;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteWatchdog;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dom4j.Document;
import org.dom4j.DocumentHelper;
import org.dom4j.Element;
import org.dom4j.io.OutputFormat;
import org.dom4j.io.XMLWriter;

public class MultiSpotbugsTestRun {

	public static void main(String[] args) throws Exception {
		String cpRoot = "I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full\\";
		multiRun(cpRoot);
//		spotbugsTest(new File(cpRoot+"cp12"));
		

	}
	
	public static void multiRun(String cpRoot) throws IOException {
		File[] cpList = (new File(cpRoot)).listFiles();
		System.out.println(cpList.length);
		for(File cpDir : cpList) {	
			System.out.println(cpDir.getName());
			spotbugsTest(cpDir);
		}
	}
	
	public static void spotbugsTest(File cpDir) throws IOException {
		String cpPath = cpDir.getAbsolutePath()+"\\";
		ArrayList<Pair<String, String>> diffFiles = new ArrayList<Pair<String,String>>();
		File diffFile = new File(cpDir.getAbsoluteFile()+"\\diffs.txt");
		List<String> lines = FileUtils.readLines(diffFile, "utf-8");
		lines.remove(0);
		lines.remove(0);					
		for(String line : lines) {
			String[] srcLines = line.split(";")[0].split("/");
			String[] dstLines = line.split(";")[1].split("/");
			String srcName = srcLines[srcLines.length-1];
			srcName = srcName.split("\\.")[0];			
			String dstName = dstLines[dstLines.length-1];
			dstName = dstName.split("\\.")[0];
//			System.out.println(srcName);
			Pair<String, String> pair = Pair.of(srcName, dstName);
			diffFiles.add(pair);	
		}
		String xmlPath = cpPath + "filter.xml";		
		try {
			creatXML(diffFiles, xmlPath);
			runSpotTest(cpDir.getAbsolutePath());
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private static void runSpotTest(String classPath) throws Exception {
		String plugin = "D:\\Program Files\\spotbugs-4.4.0\\plugin\\findsecbugs-plugin-1.11.0.jar";
		String line = "cmd.exe /C spotbugs -textui -effort:max -pluginList "+plugin+" -sourcePath "+classPath+" > sec_output.txt";
		CommandLine cmdLine = CommandLine.parse(line);
		DefaultExecutor executor = new DefaultExecutor();
		ExecuteWatchdog watchdog = new ExecuteWatchdog(1800000);//timeout 30min
		executor.setWatchdog(watchdog);
		executor.setWorkingDirectory(new File(classPath));
		executor.setExitValue(0);
		executor.execute(cmdLine);
	}
	
	
	private static void creatXML(ArrayList<Pair<String, String>> diffFiles, String xmlPath) throws IOException {
		//创建dom树
		Document doc = DocumentHelper.createDocument();
		Element root = doc.addElement("FindBugsFilter", "https://github.com/spotbugs/filter/3.0.0");		
		root.addAttribute("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance");
		root.addAttribute("xsi:schemaLocation", 
				"https://github.com/spotbugs/filter/3.0.0 https://raw.githubusercontent.com/spotbugs/spotbugs/3.1.0/spotbugs/etc/findbugsfilter.xsd");
		Element match = root.addElement("Match");		
		
		for(Pair<String, String> pair : diffFiles) {
			String srcName = pair.getLeft();
			String dstName = pair.getRight();			
			if(srcName.equals(dstName)) {
				Element source = match.addElement("Source");
				source.addAttribute("name", srcName);
			}else {
				Element source1 = match.addElement("Source");
				source1.addAttribute("name", srcName);
				Element source2 = match.addElement("Source");
				source2.addAttribute("name", dstName);
			}
		}
		
		//dom树输出XML 使用流输出
		FileOutputStream out = new FileOutputStream(xmlPath);
		//使用格式工具将字符转化
		OutputFormat fmt = OutputFormat.createPrettyPrint();//按照漂亮的格式打印出来
		fmt.setEncoding("UTF-8");
		//XML输出工具
		XMLWriter writer = new XMLWriter(out,fmt);
		writer.write(doc);
		writer.flush();
		writer.close();//关闭流，即关闭底层的c语言操作执行器
	}
	
	
	
	
	
	
	
	
	
	
	
	
}
