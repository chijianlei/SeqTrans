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

package gumtreediff.matchers.heuristic.gt;

import java.util.List;

import gumtreediff.matchers.Mapping;
import gumtreediff.matchers.MappingStore;
import gumtreediff.tree.ITree;
import gumtreediff.utils.StringAlgorithms;

public final class ParentsMappingComparator extends AbstractMappingComparator {

    public ParentsMappingComparator(List<Mapping> ambiguousMappings, MappingStore mappings, int maxTreeSize) {
        super(ambiguousMappings, mappings, maxTreeSize);
        for (Mapping ambiguousMapping: ambiguousMappings)
            similarities.put(ambiguousMapping, similarity(ambiguousMapping.getFirst(), ambiguousMapping.getSecond()));
    }

    @Override
    protected double similarity(ITree src, ITree dst) {
        return 100D * parentsJaccardSimilarity(src, dst)
                + 10D * posInParentSimilarity(src, dst) + numberingSimilarity(src , dst);
    }

    protected double parentsJaccardSimilarity(ITree src, ITree dst) {
        List<ITree> srcParents = src.getParents();
        List<ITree> dstParents = dst.getParents();
        double numerator = StringAlgorithms.lcss(srcParents, dstParents).size();
        double denominator = (double) srcParents.size() + (double) dstParents.size() - numerator;
        return numerator / denominator;
    }

}