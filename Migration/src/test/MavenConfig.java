package test;

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

public class MavenConfig {

    /**
     * @author chi
     * @version add ignore failure
     */

    public static void main(String[] args) throws Exception {
        String cpRoot = "D:\\Program Files\\languagetool-master";
        searchURL(cpRoot);
    }

    public static void searchURL(String cpRoot) throws Exception {
        File cpDir = new File(cpRoot);
        System.out.println(cpDir.getName());
        Collection<File> collections = FileUtils.listFiles(cpDir,
                FileFilterUtils.suffixFileFilter("pom.xml"), DirectoryFileFilter.INSTANCE);
        ArrayList<File> filters = new ArrayList<>(collections);
        for(File mavenBuild : filters) {
            addTextContent(mavenBuild);
//						replaceTextContent(mavenBuild);
        }
    }


    /**
     * ���failure����
     * @throws IOException
     */
    public static void addTextContent(File replaceFile) throws IOException{
        SAXReader reader = new SAXReader();
        try {
            Document read = reader.read(replaceFile);
//            // ��ȡ���ڵ�
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

            //[7]��document���������.xml�ļ��м��ɡ�
            Writer writer=new FileWriter(replaceFile);
            read.write(writer);
            //writer���Լ������ģ������Ҫ�رգ�
            //�رմ򿪵���Դ��
            writer.close();
        } catch (DocumentException e) {
            e.printStackTrace();
        }
    }

    public static ArrayList<Element> getNodes(Element node, ArrayList<Element> nodeList){
//			System.out.println("--------------------");
//			//��ǰ�ڵ�����ơ��ı����ݺ�����
//			System.out.println("��ǰ�ڵ����ƣ�"+node.getName());//��ǰ�ڵ�����
//			System.out.println("��ǰ�ڵ�����ݣ�"+node.getTextTrim());//��ǰ�ڵ�����
//			List<Attribute> listAttr=node.attributes();//��ǰ�ڵ���������Ե�list
//			for(Attribute attr:listAttr){//������ǰ�ڵ����������
//				String name=attr.getName();//��������
//				String value=attr.getValue();//���Ե�ֵ
//				System.out.println("�������ƣ�"+name+"����ֵ��"+value);
//			}

        //�ݹ������ǰ�ڵ����е��ӽڵ�
        List<Element> listElement=node.elements();//����һ���ӽڵ��list
        nodeList.addAll(listElement);
        for(Element e:listElement){//��������һ���ӽڵ�
            getNodes(e, nodeList);//�ݹ�
        }

        return nodeList;
    }
}

