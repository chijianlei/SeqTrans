package test;

import java.io.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.dom4j.Attribute;
import org.dom4j.Document;
import org.dom4j.DocumentException;
import org.dom4j.Element;
import org.dom4j.io.SAXReader;
/**
 * @author liu
 * @version 创建时间：2018年3月26日 上午11:22:29
 * 使用DOM4J解析xml文件
 */

public class test2 { 
    public static void main(String[] args) throws IOException { 
        SAXReader reader = new SAXReader(); 
        try { 
            Document read = reader.read("I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full"
            		+ "\\cp1069\\4ed66e5838476e575a83c3cd13fffb37eefa2f48\\jenkins\\core\\pom.xml"); 
//            // 获取根节点 
            Element root = read.getRootElement(); 
            ArrayList<Element> nodeList = new ArrayList<Element>();
            nodeList = getNodes(root, nodeList);
            for(Element e : nodeList) {
            	if(e.getText().equals("maven-surefire-plugin")) {
            		System.out.println("find it");
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
            Writer writer=new FileWriter(new File("I:\\20210714-Srqtrans_testcase\\Vulnerability_testset_full"
            		+ "\\cp1069\\4ed66e5838476e575a83c3cd13fffb37eefa2f48\\jenkins\\core\\pom1.xml"));
            read.write(writer);
            //writer是自己创建的，最后还需要关闭：
            //关闭打开的资源：
            writer.close();
        } catch (DocumentException e) { 
            e.printStackTrace(); 
        } 
    }
    
    public static ArrayList<Element> getNodes(Element node, ArrayList<Element> nodeList){
//		System.out.println("--------------------");		
//		//当前节点的名称、文本内容和属性
//		System.out.println("当前节点名称："+node.getName());//当前节点名称
//		System.out.println("当前节点的内容："+node.getTextTrim());//当前节点名称
//		List<Attribute> listAttr=node.attributes();//当前节点的所有属性的list
//		for(Attribute attr:listAttr){//遍历当前节点的所有属性
//			String name=attr.getName();//属性名称
//			String value=attr.getValue();//属性的值
//			System.out.println("属性名称："+name+"属性值："+value);
//		}
		
		//递归遍历当前节点所有的子节点
		List<Element> listElement=node.elements();//所有一级子节点的list
		nodeList.addAll(listElement);
		for(Element e:listElement){//遍历所有一级子节点
			getNodes(e, nodeList);//递归
		}
		
		return nodeList;
	}
}
