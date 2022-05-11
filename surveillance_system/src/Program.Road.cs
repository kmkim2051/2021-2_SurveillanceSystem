using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace surveillance_system
{
   public partial class Program
    {
        public class Road
        {
            public double[] laneVector;

            // 가로 도로 좌표
            // 첫번째 인덱스 = 도로 번호,  두번째 인덱스 = y 값
            public double[,] lane_h; // 가로 - 중앙선 y 값 
            public double[,] lane_h_upper; // 가로 - 중앙선 위 라인 y값 
            public  double[,] lane_h_lower; // 가로 - 중앙선 아래 라인 y값 

            // 세로 도로 좌표
            // 첫번째 인덱스 = 도로 번호,  두번째 인덱스 = x 값
            public double[,] lane_v; // 세로 - 중앙선 x값
            public double[,] lane_v_left; // 세로- 중앙선 왼쪽 라인 x값
            public double[,] lane_v_right; // 세로 중앙선 오른쪽 라인 x값

            public double[,] DST; // 도로 교차점
            public double[,] intersectionArea; // 도로 교차구간
            public int mapSize;
            public int lane_num;
            public int interval;
            public int width;

            public void roadBuilder(int wd, int intvl, int n_interval, int n_cctv, int n_ped)
            {
                this.lane_num = n_interval + 1;
                DST = new double[lane_num * lane_num, 2];
                intersectionArea = new double[lane_num * lane_num, 4];
                this.interval = intvl;
                this.width = wd;

                this.mapSize = n_interval * intvl + wd * lane_num;
                // 교차점, 교차구간 설정
                int idx = 0;
                for (int i = 0; i < lane_num; i++)
                {
                    for (int j = 0; j < lane_num; j++)
                    {
                        DST[idx, 0] = (intvl + wd) * i + (wd / 2);
                        DST[idx, 1] = (intvl + wd) * j + (wd / 2);

                        intersectionArea[idx, 0] = DST[idx, 0] - (wd / 2); // x_min
                        intersectionArea[idx, 1] = DST[idx, 0] + (wd / 2); // x_max
                        intersectionArea[idx, 2] = DST[idx, 1] - (wd / 2); // y_min
                        intersectionArea[idx, 3] = DST[idx, 1] + (wd / 2); // y_max
                        idx++;
                    }
                }

                // 도로 벡터 초기화
                double incr = 100;
                int laneVectorSize = (int)((intvl + wd) * (n_interval) / incr);
                //Console.WriteLine("laneSize = {0}", laneSize);
                laneVector = new double[laneVectorSize];

                for (int i = 0; i < laneVectorSize; i++)
                {
                    laneVector[i] = i * incr;
                }

                // 가로 도로 좌표 설정
                lane_h = new double[lane_num, 1];
                lane_h_upper = new double[lane_num, 1];
                lane_h_lower = new double[lane_num, 1];

                for (int i = 0; i < lane_num; i++)
                {
                    lane_h[i, 0] = i * (intvl + wd) + (wd / 2);
                    lane_h_upper[i, 0] = lane_h[i, 0] + wd / 2;
                    lane_h_lower[i, 0] = lane_h[i, 0] - wd / 2;
                }

                // 세로 도로 좌표 설정
                lane_v = new double[lane_num, 1];
                lane_v_left = new double[lane_num, 1];
                lane_v_right = new double[lane_num, 1];
                for (int i = 0; i < lane_num; i++)
                {
                    lane_v[i, 0] = i * (intvl + wd) + (wd / 2);
                    lane_v_left[i, 0] = lane_h[i, 0] - wd / 2;
                    lane_v_right[i, 0] = lane_h[i, 0] + wd / 2;
                }

                setCCTV(n_cctv, wd, lane_num);
                setPed(n_ped);
            }

            // 보행자 위치 처음 설정
            public void setPed(int n_ped)
            {
                for(int i = 0; i < n_ped; i++)
                {
                    Random rand = new Random();
                    int intersectidx = rand.Next(8);
                    double[,] newPos = getPointOfAdjacentRoad(intersectidx);
                    peds[i].X = Math.Round(newPos[0, 0]);
                    peds[i].Y = Math.Round(newPos[0, 1]);

                    //Random rand = new Random();
                    //double opt = rand.NextDouble();

                    //if (opt > 0.5) {
                    //    peds[i].X = Math.Round(laneVector.Max() * opt);
                    //    peds[i].Y = lane_h[rand.Next(0, lane_h.GetLength(0)), 0];
                    //}
                    //else
                    //{
                    //    peds[i].X =lane_v[rand.Next(0, lane_v.GetLength(0)), 0];
                    //    peds[i].Y = Math.Round(laneVector.Max() * opt);
                    //}
                }
            }

            public void setCCTV(int n_cctv, int wd, int n_interval)
            {

                double range = mapSize - width;
                int rootN = (int)Math.Sqrt((double)n_cctv);

                // x좌표가 int 형식이라 캐스팅해서 완벽한 그리드는 아닐 수 있음
                int intvl = (int)range / (rootN-1); 
                Console.WriteLine("range rootN intvl {0} {1} {2}", range, rootN, intvl);
                double startX = DST[0, 0];
                double startY = DST[0, 1];

                int cctvIdx = 0;
                for(int i = 0; i < rootN; i ++)
                {
                    startX = DST[0, 0];
                    for (int j = 0; j < rootN; j++)
                    {
                        cctvs[cctvIdx].X = (int)startX;
                        cctvs[cctvIdx].Y = (int)startY;
                        cctvIdx++;
                        startX += intvl;
                    }

                    startY += intvl;
                }
            }

            public double[,] getPointOfAdjacentRoad(int currAreaIdx)
            {
                if(currAreaIdx == -1)
                {
                    return new double[,] { { 0, 0 } };
                }

                int i, j;
                Random rand = new Random();

                do
                {
                    i = currAreaIdx / lane_num;
                    j = currAreaIdx % lane_num;

                    int opt = rand.Next(0, 4);
                    if (opt == 0) j += 1; // up
                    else if(opt == 1) j -= 1; // down
                    else if (opt == 2)  i -= 1; // left
                    else if(opt == 3) i += 1; // right

                } while (i< 0 || i >= lane_num || j < 0|| j >= lane_num);

                int idx = lane_num * i + j;
                double[,] newPos = new double[1, 2];
                newPos[0,0] = DST[idx, 0] + rand.Next(-width, width) * rand.NextDouble();
                newPos[0,1] = DST[idx, 1] + rand.Next(-width, width) * rand.NextDouble();

                return newPos;
            }

            public int getIdxOfIntersection(double x, double y)
            {
                for(int i = 0; i < intersectionArea.GetLength(0);  i++)
                {
                    if(x>=intersectionArea[i,0] && x<=intersectionArea[i,1] && y>=intersectionArea[i,2] && y <= intersectionArea[i, 3])
                    {
                        return i;
                    }
                }

                return -1;
            }

            public void printRoadInfo()
            {
                Console.WriteLine("\n======================== DST ===============================================");
                for (int i = 0; i < DST.GetLength(0); i++)
                {
                    for (int j = 0; j < 2; j++)
                    {
                        if (j == 0) Console.Write("DST[{0}].X = {1}      ", i, DST[i, j]);
                        if (j == 1) Console.Write("DST[{0}].Y = {1}      ", i, DST[i, j]);
                    }
                    Console.WriteLine();
                }
                Console.WriteLine("\n===========================================================================\n");

                /* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
                   Console.WriteLine("\n========================lane Vector========================================");
                   for (int i = 0; i < laneVector.Length; i++)
                   {
                       Console.Write(laneVector[i] + "  ");
                       if (i % 99 == 0) Console.WriteLine();
                   }
                   Console.WriteLine("\n===========================================================================\n");
                ------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

                Console.WriteLine("\n========================lane horizontal======================================");
                for (int i = 0; i < lane_h.GetLength(0); i++)
                {
                    Console.WriteLine("\n{0}번째 가로 도로 정보", i);
                    Console.WriteLine("y좌표 : 위 - 중앙 - 아래");
                    Console.WriteLine("       {0}   {1}   {2}", lane_h_upper[i,0],lane_h[i,0],lane_h_lower[i,0]);
                }
                Console.WriteLine("\n===========================================================================\n");

                

                Console.WriteLine("\n========================lane vertical========================================");
                for (int i = 0; i < lane_h.GetLength(0); i++)
                {
                    Console.WriteLine("\n{0}번째 세로 도로 정보", i);
                    Console.WriteLine("x좌표 : 왼쪽 - 중앙 - 오른쪽");
                    Console.WriteLine("       {0}   {1}   {2}", lane_v_left[i, 0], lane_v[i, 0], lane_v_right[i, 0]);
                }
                Console.WriteLine("\n===========================================================================\n");
            }
        }
    }
}