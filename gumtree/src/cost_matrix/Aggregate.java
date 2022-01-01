package cost_matrix;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
public class Aggregate {

	public static void CreatSet(Map<Integer, Integer> arr1,Map<Integer, Integer> arr2){
		 // 测试union
//        int[] arr1 = { 123, 45, 1234 };
//        int[] arr2 = { 123, 33, 45, 4, 123 };
//        Integer[] result_union = union(arr1, arr2);
//        System.out.println("求并集的结果如下：");
//        for (int str : result_union)
//        {
//            System.out.println(str);
//        }
//        System.out.println("---------------------可爱的分割线------------------------");
        // 测试insect
		List<Integer> result_insect = intersect(arr1, arr2);
        System.out.println("求交集的结果如下：");
        for (int str : result_insect)
        {
            System.out.println(str);
        }
        System.out.println("---------------------可爱的分割线------------------------");
    }

    // 求两个数值数组的并集，利用set的元素唯一性
    public static Integer[] union(int[] arr1, int[] arr2)
    {
        Set<Integer> set = new HashSet<>();
        for (int str : arr1)
        {
            set.add(str);
        }
        for (int str : arr2)
        {
            set.add(str);
        }
        Integer[] result = new Integer[set.size()];
        return set.toArray(result);
    }

    // 求两个数组的交集
    public static ArrayList<Integer> intersect(Map<Integer, Integer> arr1, Map<Integer, Integer> arr2)
    {
        Map<Integer, Boolean> map = new HashMap<>();
        ArrayList<Integer> list = new ArrayList<>();
		Iterator iter = arr1.entrySet().iterator();
		while (iter.hasNext()) {
			Map.Entry entry = (Map.Entry) iter.next();
			Integer nameInteger=(Integer)entry.getKey();
			Integer num1Integer=(Integer)entry.getValue();
			if(arr2.containsKey(nameInteger)){
				Integer num2Integer=arr2.get(nameInteger);
				int small = Math.min(num1Integer, num2Integer);
				for(int i=0;i<small;i++){
					list.add(nameInteger);
//					System.out.println(nameInteger);//输出出边序列
				 }

			}

		}
        return list;
    }

    public static int[] ConvertInteger(Integer[] arr1){
    	Integer[] integers = arr1;
    	int[] intArray = new int[arr1.length];
    	for(int i=0; i < integers.length; i ++)
    	{
    		intArray[i] = integers[i].intValue();
    	}
    	return intArray;
    }

    public static double Travel(Map<Integer, Double> map){
    	double weight = 0;
    	Iterator iter = map.entrySet().iterator();
        while(iter.hasNext()) {
                Map.Entry entry =(Map.Entry)iter.next();
                Double value = (Double)entry.getValue();
                weight = weight+value;
        }
        return weight;
    }
}

