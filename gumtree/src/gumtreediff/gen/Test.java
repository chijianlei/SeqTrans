package gumtreediff.gen;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import gumtreediff.actions.ActionGenerator;
import gumtreediff.actions.model.Action;
import gumtreediff.gen.jdt.JdtTreeGenerator;
import gumtreediff.io.ActionsIoUtils;
import gumtreediff.matchers.Matcher;
import gumtreediff.matchers.Matchers;
import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class Test {
	public static void main(String[] args) throws IOException {

        File oldFile = new File("1.java");
        File newFile = new File("2.java");

//        TreeContext oldTree = new SrcmlCppTreeGenerator().generateFromFile(oldFile);
//        TreeContext newTree = new SrcmlCppTreeGenerator().generateFromFile(newFile);
        TreeContext oldTree = new JdtTreeGenerator().generateFromFile(oldFile);
        TreeContext newTree = new JdtTreeGenerator().generateFromFile(newFile);
        Matcher m = Matchers.getInstance().getMatcher(oldTree.getRoot(), newTree.getRoot());
        m.match();

        ActionGenerator g = new ActionGenerator(oldTree.getRoot(), newTree.getRoot(), m.getMappings());
        List<Action> actions = g.generate();

        System.out.println("old:");
        System.out.println(printItree(oldTree.getRoot(), 0));
        File output = new File("output.xml");
        BufferedWriter wr = new BufferedWriter(new FileWriter(output));
        wr.append(ActionsIoUtils.toXml(oldTree, g.getActions(), m.getMappings()).toString());
        wr.close();
//        System.out.println(ActionsIoUtils.toXml(oldTree, g.getActions(), m.getMappings()).toString());
    }

    private static String printItree(ITree itree, int depth) {

        StringBuilder stringBuilder = new StringBuilder();

        stringBuilder.append("( id: ")
                .append(itree.getId())
                .append(" | ")
                .append(itree.toShortString())
                .append("\n   ");
        for (ITree child : itree.getChildren()) {
            for (int i = 0; i < depth; i++) {
                stringBuilder.append("   ");
            }
            stringBuilder.append(printItree(child, depth + 1));
        }

        return stringBuilder.toString();
    }
}
