/*
 * This file is part of GumTree.
 *
 * GumTree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GumTree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with GumTree.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2011-2015 Jean-Rémy Falleri <jr.falleri@gmail.com>
 * Copyright 2011-2015 Floréal Morandat <florealm@gmail.com> *
 */


package gumtreediff.gen.jdt;


import java.util.ArrayList;
import java.util.List;

import org.eclipse.jdt.core.dom.ASTNode;
import org.eclipse.jdt.core.dom.Assignment;
import org.eclipse.jdt.core.dom.BooleanLiteral;
import org.eclipse.jdt.core.dom.CharacterLiteral;
import org.eclipse.jdt.core.dom.InfixExpression;
import org.eclipse.jdt.core.dom.Modifier;
import org.eclipse.jdt.core.dom.Name;
import org.eclipse.jdt.core.dom.NumberLiteral;
import org.eclipse.jdt.core.dom.PostfixExpression;
import org.eclipse.jdt.core.dom.PrefixExpression;
import org.eclipse.jdt.core.dom.QualifiedName;
import org.eclipse.jdt.core.dom.StringLiteral;
import org.eclipse.jdt.core.dom.StructuralPropertyDescriptor;
import org.eclipse.jdt.core.dom.TagElement;
import org.eclipse.jdt.core.dom.TextElement;
import org.eclipse.jdt.core.dom.Type;

public class JdtVisitor  extends AbstractJdtVisitor {
    public JdtVisitor() {
        super();
    }

    @Override
    public void preVisit(ASTNode n) {
        pushNode(n, getLabel(n));
    	String type = n.getClass().getSimpleName();
//    	System.out.println(type);
//        if(type.equals("Block")) {
//        	System.out.println("size:"+getChildren(n).size());
//        }
    }

    public static List<ASTNode> getChildren(ASTNode node) {
        List<ASTNode> children = new ArrayList<>();
        List list = node.structuralPropertiesForType();
        for (Object element : list) {
            Object child = node.getStructuralProperty((StructuralPropertyDescriptor)element);
            if (child instanceof ASTNode) {
                children.add((ASTNode) child);
            }
        }
        return children;
    }


    protected String getLabel(ASTNode n) {
        if (n instanceof Name) return ((Name) n).getFullyQualifiedName();
        if ((n instanceof Type) || (n instanceof Modifier)) return n.toString();
        if (n instanceof StringLiteral) return ((StringLiteral) n).getEscapedValue();
        if (n instanceof NumberLiteral) return ((NumberLiteral) n).getToken();
        if (n instanceof CharacterLiteral) return ((CharacterLiteral) n).getEscapedValue();
        if (n instanceof BooleanLiteral) return ((BooleanLiteral) n).toString();
        if (n instanceof InfixExpression) return ((InfixExpression) n).getOperator().toString();
        if (n instanceof PrefixExpression) return ((PrefixExpression) n).getOperator().toString();
        if (n instanceof PostfixExpression) return ((PostfixExpression) n).getOperator().toString();
        if (n instanceof Assignment) return ((Assignment) n).getOperator().toString();
        if (n instanceof TextElement) return n.toString();
        if (n instanceof TagElement) return ((TagElement) n).getTagName();

        return "";
    }

    @Override
    public boolean visit(TagElement e) {
        return true;
    }

    @Override
    public boolean visit(QualifiedName name) {
        return false;
    }

    @Override
    public void postVisit(ASTNode n) {
        popNode();
    }
}
