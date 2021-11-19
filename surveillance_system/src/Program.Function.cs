using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

namespace surveillance_system
{
    partial class Program
    {
        public static void Calc_Dist_and_get_MinDist(double[] List_Pos1_X,
                                              double[] List_Pos1_Y,
                                              double[] Pos2_X,
                                              double[] Pos2_Y,
                                              ref double[] Dist_Map, // for multiple return
                                              ref double minDist,
                                              ref int Idx_minDist
                                            )
        {
            // N_Pos1 = length(List_Pos1_X);
            int N_Pos1 = List_Pos1_X.Length;
            // % N_Pos2 = length(Pos2_X);
            // int N_Pos2 = Pos2_X.Length;

            // Dist_Map = zeros(1, N_Pos1);
            for (int i = 0; i < Dist_Map.Length; i++)
            {
                Dist_Map[i] = Math.Sqrt((List_Pos1_X[i] - Pos2_X[i]) * (List_Pos1_X[i] - Pos2_X[i])
                                        + (List_Pos1_Y[i] - Pos2_Y[i]) * (List_Pos1_Y[i] - Pos2_Y[i]));
            }
            // parfor i = 1:N_Pos1
            //         Dist_Map(i) = sqrt( (List_Pos1_X(i)-Pos2_X)^2 + (List_Pos1_Y(i)-Pos2_Y)^2 );  
            // end
            minDist = Dist_Map.Min();
            Idx_minDist = Array.IndexOf(Dist_Map, minDist);
            // minDist = min(Dist_Map);
            // Idx_minDist = find(Dist_Map == minDist);
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
