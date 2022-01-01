package cost_matrix;
import java.util.Arrays;

public class KuhnMunkres {

    private static int maxN;
	private static int n;
	private static int lenX;
	private static int lenY;
    private static double[][] weights;
    private static boolean[] visitX;
	private static boolean[] visitY;
    private static double[] lx;
	private static double[] ly;
    private static double[] slack;
    private static int[] match;

    public static void main(String[] args) throws Exception{
		KuhnMunkres graph = new KuhnMunkres(5);
		int weight[][]={{3,4,6,4,9},{6,4,5,3,8},{7,5,3,4,2},{6,3,2,2,5}};
		double[] result = new double[5];
		int[][] re = getMaxBipartie(weight,result);
		int len = Math.min(lenX, lenY);
		System.out.println("len:"+len);
		for(int i=0;i<len;i++){
			System.out.println(re[i][0]+","+re[i][1]);
		}
   }


	public KuhnMunkres( int maxN )
    {
        KuhnMunkres.maxN = maxN;
        visitX = new boolean[maxN];
        visitY = new boolean[maxN];
        lx = new double[maxN];
        ly = new double[maxN];
        slack = new double[maxN];
        match = new int[maxN];
    }

    public static int[][] getMaxBipartie( int weight[][], double[] result )
    {
        if( !preProcess(weight) )
        {
            result[0] = 0.0;
            return null;
        }
        //initialize memo data for class
        //initialize label X and Y
        Arrays.fill(ly, 0);
        Arrays.fill(lx, 0);
        for( int i=0; i<n; i++ )
        {
            for( int j=0; j<n; j++ )
            {
                if( lx[i]<weights[i][j])
                    lx[i] = weights[i][j];
            }
        }

        //find a match for each X point
        for( int u=0; u<n; u++ )
        {
            Arrays.fill(slack, 0x7fffffff);
            while(true)
            {
                Arrays.fill(visitX, false);
                Arrays.fill(visitY, false);
                if( findPath(u) )   //if find it, go on to the next point
                    break;
                //otherwise update labels so that more edge will be added in
                double inc = 0x7fffffff;
                for( int v=0; v<n; v++ )
                {
                    if( !visitY[v] && slack[v] < inc )
                        inc = slack[v];
                }
                for( int i=0; i<n; i++ )
                {
                    if( visitX[i] )
                        lx[i] -= inc;
                    if( visitY[i] )
                        ly[i] += inc;
                }
            }
        }
        result[0] = 0.0;
        for( int i=0; i<n; i++ )
        {
            if( match[i] >= 0 )
                result[0] += weights[match[i]][i];
        }
        return matchResult();
    }

    public static int[][] matchResult()
    {
        int len = Math.min(lenX, lenY);
        int[][] res = new int[len][2];
        int count=0;
        for( int i=0; i<lenY; i++ )
        {
            if( match[i] >=0 && match[i]<lenX )
            {
                res[count][0] = match[i];
                res[count++][1] = i;
            }
        }
        return res;
    }

    private static boolean preProcess( int[][] weight )
    {
        if( weight == null )
            return false;
        lenX = weight.length; lenY = weight[0].length;
        if( lenX>maxN || lenY>maxN )
            return false;
        Arrays.fill(match, -1);
        n = Math.max(lenX, lenY);
        weights = new double[n][n];
        for( int i=0; i<n; i++ )
            Arrays.fill(weights[i], 0.0);
        for( int i=0; i<lenX; i++ )
            for( int j=0; j<lenY; j++ )
                weights[i][j] = weight[i][j];
        return true;
    }

    private static boolean findPath( int u )
    {
        visitX[u] = true;
        for( int v=0; v<n; v++ )
        {
            if( !visitY[v] )
            {
                double temp = lx[u]+ly[v]-weights[u][v];
                if( temp == 0.0 )
                {
                    visitY[v] = true;
                    if( match[v] == -1 || findPath(match[v]) )
                    {
                        match[v] = u;
                        return true;
                    }
                }
                else
                    slack[v] = Math.min(slack[v], temp);
            }
        }
        return false;
    }

    public int getlenX(){
		return KuhnMunkres.lenX;
	}

    public int getlenY(){
		return KuhnMunkres.lenY;
	}

}