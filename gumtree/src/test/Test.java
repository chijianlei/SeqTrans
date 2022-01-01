package test;

import java.io.File;
import java.util.List;

import gumtreediff.gen.srcml.SrcmlCppTreeGenerator;
import gumtreediff.matchers.MappingStore;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class Test {

	public static void main (String args[]) throws Exception{
		String path = "talker.cpp";
		File cppfile = new File(path);
		TreeContext tc1 = new SrcmlCppTreeGenerator().generateFromFile(cppfile);
		ITree root1 = tc1.getRoot();
		String path2 = "talker2.cpp";
		File cppfile2 = new File(path2);
		TreeContext tc2 = new SrcmlCppTreeGenerator().generateFromFile(cppfile2);
		ITree root2 = tc2.getRoot();
        Matcher m = Matchers.getInstance().getMatcher(tc1.getRoot(), tc2.getRoot());
        m.match();
        MappingStore mappings = m.getMappings();
		List<ITree> list1 = root1.getDescendants();
		for(ITree tmp : list1) {
			if(tmp.getId()==87) {
				ITree dst = mappings.getDst(tmp);
				if(tmp.isIsomorphicTo(dst))
					System.out.println("Is iso");
				else
					System.out.println("Not iso");
				String type1 = tc1.getTypeLabel(tmp);
				String type2 = tc2.getTypeLabel(dst);
				String value1 = tmp.getLabel();
				String value2 = dst.getLabel();
				System.out.println(type1+","+type2+","+value1+","+value2);
				if(type1.equals(type2)&&value1.equals(value2)) {
					System.out.println(true);
				}else {
					System.out.println(false);
				}
			}
		}

		String test1 = "";
		String test2 = null;
		if(test1.equals(test2))
			System.out.println(true);
		else
			System.out.println(false);
		String test3 = "test st";
		test3 = test3.substring(0, test3.length()-1);
		System.out.println(test3);
	}


}
