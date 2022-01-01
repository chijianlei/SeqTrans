package collect.testcase;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.dom4j.Document;
import org.dom4j.DocumentException;
import org.dom4j.Element;
import org.dom4j.io.SAXReader;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

/**
 * @author chi
 * @version add ignore failure
 */

public class ModifyMavenBuild {
	
	public static void main(String[] args) throws Exception {
		String cpRoot = "D:\\Program Files\\languagetool-master";
		searchURL(cpRoot);
	}

	public static void searchURL(String cpRoot) throws Exception {
		File[] cpList = (new File(cpRoot)).listFiles();
		ArrayList<String> noMavenList = new ArrayList<String>();
		for(File cpFile : cpList) {
			File[] cpDirs = cpFile.listFiles();
			for(File cpDir : cpDirs) {
				if(cpDir.isDirectory()) {
//					if(!cpFile.getName().equals("cp102"))
//						continue;
					System.out.println(cpDir.getName());
					Collection<File> collections = FileUtils.listFiles(cpDir, 
							FileFilterUtils.suffixFileFilter("pom.xml"), DirectoryFileFilter.INSTANCE);
					ArrayList<File> filters = new ArrayList<>(collections);
					if(filters.size()==0) {
						noMavenList.add(cpFile.getName());
					}
					System.out.println(cpFile.getName()+": "+filters.size());
					for(File mavenBuild : filters) {
						addTextContent(mavenBuild);
//						replaceTextContent(mavenBuild);
					}
				}
			}
		}
		
		for(String no : noMavenList) {
			System.out.println(no);
		}
	}
	

     /**
	 * 添加failure忽略
	 * @param path
	 * @throws IOException
	 */
	public static void addTextContent(File replaceFile) throws IOException{
		SAXReader reader = new SAXReader(); 
        try { 
            Document read = reader.read(replaceFile); 
//            // 获取根节点 
            Element root = read.getRootElement(); 
            ArrayList<Element> nodeList = new ArrayList<Element>();
            nodeList = getNodes(root, nodeList);
            for(Element e : nodeList) {
            	if(e.getText().equals("maven-surefire-plugin")) {
            		System.out.println("find test");
            		Element par = e.getParent();
            		Iterator eles = par.elementIterator(); 
            		while(eles.hasNext()) { 
            			Element child = (Element)eles.next();
            			if(child.getName().equals("configuration")) {
            				Element addElement = child.addElement("testFailureIgnore");
            				addElement.addText("true");
            			}
            		}
            	}
            }
            
          //[7]将document对象输出到.xml文件中即可。
            Writer writer=new FileWriter(replaceFile);
            read.write(writer);
            //writer是自己创建的，最后还需要关闭：
            //关闭打开的资源：
            writer.close();
        } catch (DocumentException e) { 
            e.printStackTrace(); 
        } 
	}
	
	   public static ArrayList<Element> getNodes(Element node, ArrayList<Element> nodeList){
//			System.out.println("--------------------");		
//			//当前节点的名称、文本内容和属性
//			System.out.println("当前节点名称："+node.getName());//当前节点名称
//			System.out.println("当前节点的内容："+node.getTextTrim());//当前节点名称
//			List<Attribute> listAttr=node.attributes();//当前节点的所有属性的list
//			for(Attribute attr:listAttr){//遍历当前节点的所有属性
//				String name=attr.getName();//属性名称
//				String value=attr.getValue();//属性的值
//				System.out.println("属性名称："+name+"属性值："+value);
//			}
			
			//递归遍历当前节点所有的子节点
			List<Element> listElement=node.elements();//所有一级子节点的list
			nodeList.addAll(listElement);
			for(Element e:listElement){//遍历所有一级子节点
				getNodes(e, nodeList);//递归
			}
			
			return nodeList;
		}
}
