using System;
using System.Collections.Generic;
using System.Text;

namespace surveillance_system
{
    public partial class Program
    {
      public class FOV
      {
        public double[] X0;
        public double[] X1;
        public double[] X2;
        
        public double[] Y0;
        public double[] Y1;
        public double[] Y2;

        public double[] Z0;
        public double[] Z1;
        public double[] Z2;


      }
        public class CCTV
        {
            public int X;

            public int Y;

            public int Z;

            public double WD;

            public double HE;

            public double H_AOV;

            public double V_AOV;

            public int imW;

            public int imH;

            public double Focal_Length;

            public double ViewAngleH;

            public double ViewAngleV;

            public double MAX_Dist_X;

            public double MAX_Dist_Y;

            public double Direction;

            public double[] H_FOV_X0;

            public double[] H_FOV_X1;

            public double[] H_FOV_X2;

            public double[] H_FOV_Y0;

            public double[] H_FOV_Y1;

            public double[] H_FOV_Y2;

            public double[] H_FOV_Z0;

            public double[] H_FOV_Z1;

            public double[] H_FOV_Z2;

            public double[] V_FOV_X0;

            public double[] V_FOV_X1;

            public double[] V_FOV_X2;

            public double[] V_FOV_Y0;

            public double[] V_FOV_Y1;

            public double[] V_FOV_Y2;

            public double[] V_FOV_Z0;

            public double[] V_FOV_Z1;

            public double[] V_FOV_Z2;

            public void get_H_FOV(
                double[] Dist,
                double HE,
                double Focal_Length,
                double ViewAngle,
                double X,
                double Y
            )
            {
                double[] H_FOV_0 = new double[Dist.Length];
                double[] H_FOV_1 = new double[Dist.Length];
                double[] H_FOV_2 = new double[Dist.Length];

                double[] H_X0 = new double[Dist.Length];
                double[] H_X1 = new double[Dist.Length];
                double[] H_X2 = new double[Dist.Length];

                double[] H_Y0 = new double[Dist.Length];
                double[] H_Y1 = new double[Dist.Length];
                double[] H_Y2 = new double[Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    H_FOV_0[i] = 0;
                    H_FOV_1[i] = (1 / 2) * Dist[i] * HE / Focal_Length;
                    H_FOV_2[i] = (-1 / 2) * Dist[i] * HE / Focal_Length;

                    H_X0[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        H_FOV_0[i] * Math.Sin(ViewAngle);
                    H_X1[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        H_FOV_1[i] * Math.Sin(ViewAngle);
                    H_X2[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        H_FOV_2[i] * Math.Sin(ViewAngle);

                    H_Y0[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        H_FOV_0[i] * Math.Cos(ViewAngle);
                    H_Y1[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        H_FOV_1[i] * Math.Cos(ViewAngle);
                    H_Y2[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        H_FOV_2[i] * Math.Cos(ViewAngle);
                }
                double[,] H_FOV_X = new double[3, Dist.Length];
                double[,] H_FOV_Y = new double[3, Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    H_FOV_X[0, i] = H_X0[i] + X;
                    H_FOV_X[1, i] = H_X1[i] + X;
                    H_FOV_X[2, i] = H_X2[i] + X;
                    H_FOV_Y[0, i] = H_Y0[i] + Y;
                    H_FOV_Y[1, i] = H_Y1[i] + Y;
                    H_FOV_Y[2, i] = H_Y2[i] + Y;
                }
                H_FOV_X0 = new double[Dist.Length];
                H_FOV_X1 = new double[Dist.Length];
                H_FOV_X2 = new double[Dist.Length];

                H_FOV_Y0 = new double[Dist.Length];
                H_FOV_Y1 = new double[Dist.Length];
                H_FOV_Y2 = new double[Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    H_FOV_X0[i] = H_FOV_X[0, i];
                    H_FOV_X1[i] = H_FOV_X[1, i];
                    H_FOV_X2[i] = H_FOV_X[2, i];

                    H_FOV_Y0[i] = H_FOV_Y[0, i];
                    H_FOV_Y1[i] = H_FOV_Y[1, i];
                    H_FOV_Y2[i] = H_FOV_Y[2, i];
                }
            }

            public void get_V_FOV(
                double[] Dist,
                double WD,
                double Focal_Length,
                double ViewAngle,
                double X,
                double Z
            )
            {
                double[] V_FOV_0 = new double[Dist.Length];
                double[] V_FOV_1 = new double[Dist.Length];
                double[] V_FOV_2 = new double[Dist.Length];

                double[] V_X0 = new double[Dist.Length];
                double[] V_X1 = new double[Dist.Length];
                double[] V_X2 = new double[Dist.Length];

                double[] V_Y0 = new double[Dist.Length];
                double[] V_Y1 = new double[Dist.Length];
                double[] V_Y2 = new double[Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    V_FOV_0[i] = 0;
                    V_FOV_1[i] = (1 / 2) * Dist[i] * WD / Focal_Length;
                    V_FOV_2[i] = (-1 / 2) * Dist[i] * WD / Focal_Length;

                    V_X0[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        V_FOV_0[i] * Math.Sin(ViewAngle);
                    V_X1[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        V_FOV_1[i] * Math.Sin(ViewAngle);
                    V_X2[i] =
                        Dist[i] * Math.Cos(ViewAngle) -
                        V_FOV_2[i] * Math.Sin(ViewAngle);

                    V_Y0[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        V_FOV_0[i] * Math.Cos(ViewAngle);
                    V_Y1[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        V_FOV_1[i] * Math.Cos(ViewAngle);
                    V_Y2[i] =
                        Dist[i] * Math.Sin(ViewAngle) +
                        V_FOV_2[i] * Math.Cos(ViewAngle);
                }
                double[,] V_FOV_X = new double[3, Dist.Length];
                double[,] V_FOV_Y = new double[3, Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    V_FOV_X[0, i] = V_X0[i] + X;
                    V_FOV_X[1, i] = V_X1[i] + X;
                    V_FOV_X[2, i] = V_X2[i] + X;
                    V_FOV_Y[0, i] = V_Y0[i] + Z;
                    V_FOV_Y[1, i] = V_Y1[i] + Z;
                    V_FOV_Y[2, i] = V_Y2[i] + Z;
                }
                V_FOV_X0 = new double[Dist.Length];
                V_FOV_X1 = new double[Dist.Length];
                V_FOV_X2 = new double[Dist.Length];

                V_FOV_Y0 = new double[Dist.Length];
                V_FOV_Y1 = new double[Dist.Length];
                V_FOV_Y2 = new double[Dist.Length];

                for (int i = 0; i < Dist.Length; i++)
                {
                    V_FOV_X0[i] = V_FOV_X[0, i];
                    V_FOV_X1[i] = V_FOV_X[1, i];
                    V_FOV_X2[i] = V_FOV_X[2, i];

                    V_FOV_Y0[i] = V_FOV_Y[0, i];
                    V_FOV_Y1[i] = V_FOV_Y[1, i];
                    V_FOV_Y2[i] = V_FOV_Y[2, i];
                }
            }


            // public double Angle;

            public double[] SurvDist_H;

            public double[] SurvDist_V;

            public double[] PPM_H;

            public double[] PPM_V;

            public void get_PixelDensity(
                double[] dist,
                double WD,
                double HE,
                double Lens_FocalLength,
                int imW,
                int imH
            )
            {
                int len = dist.Length;
                double[] Dist_Meter = new double[len];
                double[] H_FOV = new double[len];
                double[] V_FOV = new double[len];
                PPM_H = new double[len];
                PPM_V = new double[len];

                for (int i = 0; i < len; i++)
                {
                    Dist_Meter[i] = dist[i] * 0.003;
                }

                for (int i = 0; i < len; i++)
                {
                    H_FOV[i] = Dist_Meter[i] * WD / Lens_FocalLength;
                    V_FOV[i] = Dist_Meter[i] * HE / Lens_FocalLength;
                }

                for (int i = 0; i < len; i++)
                {
                    PPM_H[i] = imW / H_FOV[i];
                    PPM_V[i] = imH / V_FOV[i];
                }

                int[] List_PPM_Criteria = new int[] { 250, 125, 62, 25, 12 };
                SurvDist_H = new double[List_PPM_Criteria.Length];
                SurvDist_V = new double[List_PPM_Criteria.Length];

                for (int i = 0; i < List_PPM_Criteria.Length; i++)
                {
                    double[] temp = new double[len];
                    for (int j = 0; j < len; j++)
                    {
                        temp[j] = Math.Abs(PPM_H[j] - List_PPM_Criteria[i]);
                    }

                    int idx = findMinIdx(temp);
                    SurvDist_H[i] = Dist_Meter[idx] * 0.003;

                    for (int j = 0; j < len; j++)
                    {
                        temp[j] = Math.Abs(PPM_V[j] - List_PPM_Criteria[i]);
                    }

                    idx = findMinIdx(temp);
                    SurvDist_V[i] = Dist_Meter[idx] * 0.003;
                }
            }

            public void printCCTVInfo()
            {
                Console.WriteLine("======================Info======================");
                Console.WriteLine("좌표 : ({0},{1},{2}) \n", this.X, this.Y, this.Z);
                //Console.WriteLine("카메라 센서(너비, 높이) : ({0},{1}) \n", this.WD, this.HE);
                //Console.WriteLine("imH imW: {0}, {1} \n", this.imH, this.imW);
                //Console.WriteLine("초점거리 : {0} \n", this.Focal_Length);
                Console.WriteLine("ViewAngleH : {0}  ViewAngleV: {1}  \n",this.ViewAngleH, this.ViewAngleV);
                //Console.WriteLine("H_AOV : {0}   V_AOV : {1} \n",  this.H_AOV, this.V_AOV);
            }
        }
    }
}