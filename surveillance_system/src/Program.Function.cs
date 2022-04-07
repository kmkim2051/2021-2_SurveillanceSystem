using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace surveillance_system
{
    partial class Program
    {
        public static void Calc_Dist_and_get_MinDist(double[,] List_Pos1, // 목적지 좌표 리스트
                                              double Pos2_X, // 보행자 x좌표
                                              double Pos2_Y, // 보행자 y좌표
                                              ref double[] Dist_Map, // for multiple return
                                              ref double minDist,
                                              ref int Idx_minDist
                                            )
        {
            int N_Pos1 = List_Pos1.GetLength(0);

            for (int i = 0; i < N_Pos1; i++)
            {
                Dist_Map[i] = Math.Sqrt((List_Pos1[i,0] - Pos2_X) * (List_Pos1[i,0] - Pos2_X)
                                        + (List_Pos1[i,1] - Pos2_Y) * (List_Pos1[i,1] - Pos2_Y));
            }


            minDist = Dist_Map.Min();
            Idx_minDist = Array.IndexOf(Dist_Map, minDist);
        }

        public static void getResolution(Pedestrian PED, CCTV[] CCTV)
        {
            // N_CCTV = length(CCTV.X); // why CCTV.X ..?
            int N_CCTV = CCTV.Length;
            double H_cosine0 = 0;
            double H_cosine1 = 0;
            double H_cosine2 = 0;

            double V_cosine0 = 0;
            double V_cosine1 = 0;
            double V_cosine2 = 0;

            bool H_Detected = false;
            bool V_Detected = false;

            // todo: for loop
            // for i = 1:N_CCTV
            //     [ d idx ] = min( abs(CCTV.H_FOV_X0(i)-PED.Pos_H1(1) + CCTV.X(i)) );
            //     H_Xt1 = CCTV.H_FOV_X0(idx);
            //     H_Yt1 = CCTV.H_FOV_Y0(idx);
            //     A = [H_Xt1 H_Yt1];
            //     B = [PED.Pos_H1(1)  PED.Pos_H1(2)] - [CCTV.X(i) CCTV.Y(i)];
            //     H_cosine1 = dot(A,B)/( norm(A)*norm(B));
            // end
            if (H_cosine0 <= H_cosine1 && H_cosine0 <= H_cosine2)
                H_Detected = true;
            else
                H_Detected = false;

            if (V_cosine0 <= V_cosine1 && V_cosine0 <= V_cosine2)
                V_Detected = true;
            else
                V_Detected = false;
        }


        public static int findMinIdx(double[] temp)
        {
            double min = temp[0];
            int idx = 0;
            for (int j = 1; j < temp.Length; j++)
            {
                if (temp[j] < min)
                {
                    min = temp[j];
                    idx = j;
                }
            }

            return idx;
        }
    }
}
