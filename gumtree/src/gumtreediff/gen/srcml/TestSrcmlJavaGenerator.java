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
 * Copyright 2016 Jean-Rémy Falleri <jr.falleri@gmail.com>
 */

package gumtreediff.gen.srcml;

import java.io.IOException;

import org.junit.Assert;
import org.junit.Test;

import gumtreediff.tree.ITree;
import gumtreediff.tree.TreeContext;

public class TestSrcmlJavaGenerator {

    @Test
    public void testSimple() throws IOException {
        String input = "public class HelloWorld {\n"
                + "public static void main(String[] args) {\n"
                + "System.out.println(\"Hello, World\");\n"
                + "}\n"
                + "}";
        TreeContext ctx = new SrcmlJavaTreeGenerator().generateFromString(input);
        ITree t = ctx.getRoot();
        Assert.assertEquals(33, t.getSize());
    }

}
