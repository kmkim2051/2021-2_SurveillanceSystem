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
            public int size;

            public void roadBuilder(int wd, int intvl, int n_interval, int n_cctv, int n_ped)
            {
                DST = new double[n_interval * n_interval, 2];

                // 교차점 좌표 저장
                int idx = 0;
                for (int i = 0; i < n_interval; i++)
                {
                    for (int j = 0; j < n_interval; j++)
                    {
                        DST[idx, 0] = (intvl + wd) * i;
                        DST[idx, 1] = (intvl + wd) * j;
                        idx++;
                    }
                }

                // 도로 벡터 초기화
                double incr = 100;
                int laneSize = (int)((intvl + wd) * (n_interval - 1) / incr);
                size = laneSize * (int)incr;
                //Console.WriteLine("laneSize = {0}", laneSize);
                laneVector = new double[laneSize];

                for (int i = 0; i < laneSize; i++)
                {
                    laneVector[i] = i * incr;
                }

                // 가로 도로 좌표 설정
                lane_h = new double[n_interval, 1];
                lane_h_upper = new double[n_interval, 1];
                lane_h_lower = new double[n_interval, 1];

                for (int i = 0; i < n_interval; i++)
                {
                    lane_h[i, 0] = i * (intvl + wd);
                    lane_h_upper[i, 0] = lane_h[i, 0] + wd / 2;
                    lane_h_lower[i, 0] = lane_h[i, 0] - wd / 2;
                }

                // 세로 도로 좌표 설정
                lane_v = new double[n_interval, 1];
                lane_v_left = new double[n_interval, 1];
                lane_v_right = new double[n_interval, 1];
                for (int i = 0; i < n_interval; i++)
                {
                    lane_v[i, 0] = i * (intvl + wd);
                    lane_v_left[i, 0] = lane_h[i, 0] - wd / 2;
                    lane_v_right[i, 0] = lane_h[i, 0] + wd / 2;
                }

                setCCTV(n_cctv,wd, n_interval);
                setPed(n_ped);
            }

            public void setPed(int n_ped)
            {
                for(int i = 0; i < n_ped; i++)
                {
                    Random rand = new Random();
                    double opt = rand.NextDouble();

                    if (opt > 0.5) {
                        peds[i].X = Math.Round(laneVector.Max() * opt);
                        peds[i].Y = lane_h[rand.Next(0, lane_h.GetLength(0)), 0];
                    }
                    else
                    {
                        peds[i].X =lane_v[rand.Next(0, lane_v.GetLength(0)), 0];
                        peds[i].Y = Math.Round(laneVector.Max() * opt);
                    }
                }
            }

            public void setCCTV(int n_cctv, int wd, int n_interval)
            {
                for (int i = 0; i < n_cctv; i++)
                {
                    Random rand = new Random();
                    double opt = rand.NextDouble();
                    
                    // 잘 모르겠음 
                    if (opt > 0.5)
                    {
                        double rand_y = Math.Round(opt);
                        if (rand_y == 0) rand_y = -1;

                        cctvs[i].X = (int)(-1 * wd / 2 + laneVector.Max() * opt);
                        cctvs[i].Y = (int)(lane_h[rand.Next(0, n_interval),0]+rand_y*wd/2);
                    }
                    else
                    {
                        double rand_x = Math.Round(opt);
                        if (rand_x == 0) rand_x = -1;

                        cctvs[i].X = (int)(lane_v[rand.Next(0, n_interval), 0] + rand_x * wd / 2);
                        cctvs[i].Y = (int)(-1 * wd / 2 + laneVector.Max() * opt);
                    }
                }
            }

            public void printRoadInfo()
            {
                // Console.WriteLine("\n======================== DST ===============================================");
                for (int i = 0; i < DST.GetLength(0); i++)
                {
                    // for time check
                    // for (int j = 0; j < 2; j++)
                    // {
                    //     if (j == 0) Console.Write("DST[{0}].X = {1}      ", i, DST[i, j]);
                    //     if (j == 1) Console.Write("DST[{0}].Y = {1}      ", i, DST[i, j]);
                    // }
                    // Console.WriteLine();
                }
                // Console.WriteLine("\n===========================================================================\n");

                /* -----------------------------------------------------------------------------------------------------------------------------------------------------------------------

                   Console.WriteLine("\n========================lane Vector========================================");
                   for (int i = 0; i < laneVector.Length; i++)
                   {
                       Console.Write(laneVector[i] + "  ");
                       if (i % 99 == 0) Console.WriteLine();
                   }
                   Console.WriteLine("\n===========================================================================\n");

                ------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
                //for time check
                // Console.WriteLine("\n========================lane horizontal======================================");
                // for (int i = 0; i < lane_h.GetLength(0); i++)
                // {
                //     Console.WriteLine("\n{0}번째 가로 도로 정보", i);
                //     Console.WriteLine("y좌표 : 위 - 중앙 - 아래");
                //     Console.WriteLine("       {0}   {1}   {2}", lane_h_upper[i,0],lane_h[i,0],lane_h_lower[i,0]);
                // }
                // Console.WriteLine("\n===========================================================================\n");

                

                // Console.WriteLine("\n========================lane vertical========================================");
                // for (int i = 0; i < lane_h.GetLength(0); i++)
                // {
                //     Console.WriteLine("\n{0}번째 세로 도로 정보", i);
                //     Console.WriteLine("x좌표 : 왼쪽 - 중앙 - 오른쪽");
                //     Console.WriteLine("       {0}   {1}   {2}", lane_v_left[i, 0], lane_v[i, 0], lane_v_right[i, 0]);
                // }
                // Console.WriteLine("\n===========================================================================\n");
            }
        }
    }
}