using System;
using System.Text;
using System.Collections.Generic;

namespace surveillance_system
{
    public partial class Program
    {
        // Angle 0,1,2 따로따로 3개씩 차례대로 나오는 구조를
        // Angle3D 하나로 묶어서 다루기 위함
        // (예시)
        // Before)) X0, X1, X2
        // After)) Angle3D X; 
        //         X.Angle_0, X.Angle_1, X.Angle_2;
        public class Angle3D
        {
            public double[] Angle_0;

            public double[] Angle_1;

            public double[] Angle_2;

            public Angle3D(int size)
            {
                if (size <= 0) return;
                Angle_0 = new double[size];
                Angle_1 = new double[size];
                Angle_2 = new double[size];
            }

            public void Set_Angle012(double a0, double a1, double a2, int index)
            {
                if (
                    index < 0 ||
                    index >= Angle_0.Length ||
                    index >= Angle_1.Length ||
                    index >= Angle_2.Length
                ) return;
                if (
                    Angle_0.Length == 0 ||
                    Angle_1.Length == 0 ||
                    Angle_2.Length == 0
                ) return;

                Angle_0[index] = a0;
                Angle_1[index] = a1;
                Angle_2[index] = a2;
            }
        }
        // 211126
        // H_FOV와 V_FOV를 하나의 클래스로 묶고자 작성
        // CCTV 내부의 H(V)_FOV_X(YZ)#,  시리즈: X0~2, Y0~2, Z0~2 대체
        // todo: Horizon or Vertical 구분하는 type 변수 필요?
        public class FOV
        {
            public Angle3D X;

            public double[] X0;

            public double[] X1;

            public double[] X2;

            // public Angle3D Y;
            public double[] Y0;

            public double[] Y1;

            public double[] Y2;

            // public Angle3D Z;
            public double[] Z0;

            public double[] Z1;

            public double[] Z2;

            // initialize with 'new' keyword
            public void Init_X012(int size)
            {
                if (size <= 0) return;
                X0 = new double[size];
                X1 = new double[size];
                X2 = new double[size];
                X = new Angle3D(size);
            }

            public void Init_Y012(int size)
            {
                if (size <= 0) return;
                Y0 = new double[size];
                Y1 = new double[size];
                Y2 = new double[size];
                // Y.Init_Angle012(size);
            }

            public void Init_Z012(int size)
            {
                if (size <= 0) return;
                Z0 = new double[size];
                Z1 = new double[size];
                Z2 = new double[size];
                // Z.Init_Angle012(size);
            }

            // all setter WITH index
            // set value to X0, X1, X2
            public void Set_X012(
                double X0_i,
                double X1_i,
                double X2_i,
                int index
            )
            {
                if (
                    index < 0 ||
                    index >= X0.Length ||
                    index >= X1.Length ||
                    index >= X2.Length
                ) return;
                if (X0.Length == 0 || X1.Length == 0 || X2.Length == 0) return;
                X0[index] = X0_i;
                X1[index] = X1_i;
                X2[index] = X2_i;
                // X.Set_Angle012(X0_i, X1_i, X2_i, index);
            }

            // set value to Y0, Y1, Y2
            public void Set_Y012(
                double Y0_i,
                double Y1_i,
                double Y2_i,
                int index
            )
            {
                if (
                    index < 0 ||
                    index >= Y0.Length ||
                    index >= Y1.Length ||
                    index >= Y2.Length
                ) return;
                if (Y0.Length == 0 || Y1.Length == 0 || Y2.Length == 0) return;
                Y0[index] = Y0_i;
                Y1[index] = Y1_i;
                Y2[index] = Y2_i;
                // X.Set_Angle012(X0_i, X1_i, X2_i, index);
            }

            // set value to Z0, Z1, Z2
            public void Set_Z012(
                double Z0_i,
                double Z1_i,
                double Z2_i,
                int index
            )
            {
                if (
                    index < 0 ||
                    index >= Z0.Length ||
                    index >= Z1.Length ||
                    index >= Z2.Length
                ) return;
                if (Z0.Length == 0 || Z1.Length == 0 || Z2.Length == 0) return;
                Z0[index] = Z0_i;
                Z1[index] = Z1_i;
                Z2[index] = Z2_i;
                // Z.Set_Angle012(X0_i, X1_i, X2_i, index);
            }
        }

        public class CCTV
        {
            public int X;

            public int Y;

            public int Z = 3000;

            public double WD;

            public double HE;

            public double H_AOV;

            public double V_AOV;

            public int imW;

            public int imH;

            public double Focal_Length;

            public double ViewAngleH;

            public double ViewAngleV = -35;

            public double Eff_Dist_From;
            public double Eff_Dist_To;

            // camera control
            public double Direction;

            public bool isFixed = false;

            // 최대거리
            public double Max_Dist;

            // todo
            // ground sample distance

            // new FOV class member

            public FOV H_FOV;

            public FOV V_FOV;

            public List<int> detectedPedIndex;

            public void setZ(int z)
            {
              if (z >= 3000) // 3000: minimum z
              {
                Z = z;
              }
            }
            public void setFixMode(bool mode)
            {
              isFixed = mode;
              // debug
              // Console.WriteLine("CCTV Rotation: " + isFixed.ToString());
            }
            public void setViewAngleH(double angleH)
            {
                // 양수 or 음수 예외처리x 에러 아니고 회전 방향 차이
                ViewAngleH = angleH;
            }
            public void setViewAngleV(double angleV)
            {
              // angleV default value is -35
              if(angleV > -35 || angleV < -55)
              {
                // debug
                Console.WriteLine("[Warning] Horizontal ViewAngle should be between -35 ~ -55");
                return;
              }
              ViewAngleV = angleV;
            }
            public void rotateHorizon(double rotationDegree)
            {
              // horizontal rotation
              if(!isFixed) {
                setViewAngleH((ViewAngleH + rotationDegree) % 360);
              }
            }
            
            public double calcDistToPed(Pedestrian ped)
            {
              // 이차원 상 거리
              return Math.Sqrt(Math.Pow(Math.Abs(X - ped.X),2) 
                              + Math.Pow(Math.Abs(Y - ped.Y),2));
            }
            public double calcEffDistToPed(Pedestrian ped)
            {
              // matlab code
              // CCTV.R_eff(i) = (CCTV.Z(i)-Ped_Height*1.0)/tand(abs(CCTV.ViewAngleV(i))-(CCTV.V_AOV(i)/2));
              double distance = (Z-ped.H*1.0) / Math.Tan(Math.Abs(ViewAngleV) - (V_AOV/2));
              return distance;
            }
            public double calcBlindToPed(Pedestrian ped)
            {
              // matlab code
              // CCTV.R_blind(i) = CCTV.Z(i)/tand(CCTV.V_AOV(i)/2 + abs(CCTV.ViewAngleV(i)))
              double distance = Z / Math.Tan((V_AOV/2) + Math.Abs(ViewAngleV));
              return distance;
            }
            public bool isPedInEffDist(Pedestrian ped)
            {
              return (calcDistToPed(ped) < Max_Dist) // 기기 성능에 따른 최대 감시거리 
                  && (calcDistToPed(ped) >= calcBlindToPed(ped)) // blind ~ 유효거리
                  && (calcDistToPed(ped) <= calcEffDistToPed(ped));
            }
            public void get_H_FOV(
                double[] Dist,
                double WD,
                double Focal_Length,
                double ViewAngle,
                double X,
                double Y
            )
            {
                // 211126_2
                Angle3D H_FOV_temp = new Angle3D(Dist.Length);
                Angle3D H_X_temp = new Angle3D(Dist.Length);
                Angle3D H_Y_temp = new Angle3D(Dist.Length);

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
                    H_FOV_0[i] = 0; // 해당 점이 CCTV에서 Horizontal(X)만큼 얼마나 떨어졌는지 크기
                    // 220407
                    // https://github.com/0BoOKim/Surveillance-System/blob/main/get_H_FOV.m
                    // line 9, 10
                    // H_FOV_1[i] = (1 / 2) * Dist[i] * HE / Focal_Length;
                    // H_FOV_2[i] = (-1 / 2) * Dist[i] * HE / Focal_Length;
                    H_FOV_1[i] = (1 / 2) * Dist[i] * WD / Focal_Length;
                    H_FOV_2[i] = (-1 / 2) * Dist[i] * WD / Focal_Length;

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

                H_FOV = new FOV();
                H_FOV.Init_X012(Dist.Length);
                H_FOV.Init_Y012(Dist.Length);

                // 220407 반복문을 돌 필요가 있나?
                // 그냥 배열 그대로 넘겨주면 되는거 아닌가 고민..
                for (int i = 0; i < Dist.Length; i++)
                {
                    // 211126
                    H_FOV
                        .Set_X012(H_FOV_X[0, i],
                        H_FOV_X[1, i],
                        H_FOV_X[2, i],
                        i);
                    H_FOV
                        .Set_Y012(H_FOV_Y[0, i],
                        H_FOV_Y[1, i],
                        H_FOV_Y[2, i],
                        i);
                }
            }

            public void get_V_FOV(
                double[] Dist,
                double HE,
                double Focal_Length,
                double ViewAngle,
                double X,
                double Z
            )
            {
                // 211126_2
                Angle3D V_FOV_temp = new Angle3D(Dist.Length);
                Angle3D V_X_temp = new Angle3D(Dist.Length);
                Angle3D V_Y_temp = new Angle3D(Dist.Length);

                for (int i = 0; i < Dist.Length; i++)
                {
                    // 211126_2
                    // 220407
                    // https://github.com/0BoOKim/Surveillance-System/blob/main/get_V_FOV.m
                    // 기존 코드 두줄
                    // double Dist_WD_FL = Dist[i] * WD / Focal_Length;
                    // double Dist_Cos_ViewAngle = Dist[i] * Math.Cos(ViewAngle);
                    double Dist_HE_FL = (1/2) * Dist[i] * HE / Focal_Length;
                    double Dist_Cos_ViewAngle = Dist[i] * Math.Cos(ViewAngle);
                    double Dist_Sin_ViewAngle = Dist[i] * Math.Sin(ViewAngle);


                    V_FOV_temp
                        .Set_Angle012(0, Dist_HE_FL, (-1) * Dist_HE_FL, i);
                    V_X_temp
                        .Set_Angle012(
                          Dist_Cos_ViewAngle - V_FOV_temp.Angle_0[i] * Math.Sin(ViewAngle),
                          Dist_Cos_ViewAngle - V_FOV_temp.Angle_1[i] * Math.Sin(ViewAngle),
                          Dist_Cos_ViewAngle - V_FOV_temp.Angle_2[i] * Math.Sin(ViewAngle),
                         i
                        );
                    V_Y_temp
                        .Set_Angle012(
                          Dist_Sin_ViewAngle + V_FOV_temp.Angle_0[i] * Math.Cos(ViewAngle),
                          Dist_Sin_ViewAngle + V_FOV_temp.Angle_1[i] * Math.Cos(ViewAngle),
                          Dist_Sin_ViewAngle + V_FOV_temp.Angle_2[i] * Math.Cos(ViewAngle),
                          i
                        );
                }

                // 211126_2
                Angle3D V_FOV_X_temp = new Angle3D(Dist.Length);
                Angle3D V_FOV_Y_temp = new Angle3D(Dist.Length);
                Angle3D V_FOV_Z_temp = new Angle3D(Dist.Length);

                for (int i = 0; i < Dist.Length; i++)
                {
                    // 211126_2
                    // V_FOV는 Y+Z ?
                    V_FOV_X_temp.Set_Angle012(V_X_temp.Angle_0[i] + X,
                                              V_X_temp.Angle_1[i] + X,
                                              V_X_temp.Angle_2[i] + X,
                                              i);
                    V_FOV_Y_temp.Set_Angle012(V_Y_temp.Angle_0[i] + Z,
                                              V_Y_temp.Angle_1[i] + Z,
                                              V_Y_temp.Angle_2[i] + Z,
                                              i);
                }

                V_FOV = new FOV();
                // 211126
                // 220331 Init은 생성자로 빼도 되지않나
                V_FOV.Init_X012(Dist.Length);
                V_FOV.Init_Z012(Dist.Length);

                for (int i = 0; i < Dist.Length; i++)
                {
                    // 211126_2
                    V_FOV.Set_X012(V_FOV_X_temp.Angle_0[i], 
                                  V_FOV_X_temp.Angle_1[i], 
                                  V_FOV_X_temp.Angle_2[i], 
                                  i);
                    V_FOV.Set_Z012(V_FOV_Y_temp.Angle_0[i], 
                                  V_FOV_Y_temp.Angle_1[i], 
                                  V_FOV_Y_temp.Angle_2[i], 
                                  i);                    
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
                    Dist_Meter[i] = dist[i] * 0.001;
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
                    SurvDist_H[i] = Dist_Meter[idx];

                    for (int j = 0; j < len; j++)
                    {
                        temp[j] = Math.Abs(PPM_V[j] - List_PPM_Criteria[i]);
                    }

                    idx = findMinIdx(temp);
                    SurvDist_V[i] = Dist_Meter[idx];

                    // Console.WriteLine("survdist h, v {0} {1} ", SurvDist_H[i], SurvDist_V[i]);
                }
            }

            public void printCCTVInfo()
            {
                Console
                    .WriteLine("======================Info======================");
                Console
                    .WriteLine("좌표 : ({0},{1},{2}) \n",
                    this.X,
                    this.Y,
                    this.Z);

                //Console.WriteLine("카메라 센서(너비, 높이) : ({0},{1}) \n", this.WD, this.HE);
                //Console.WriteLine("imH imW: {0}, {1} \n", this.imH, this.imW);
                //Console.WriteLine("초점거리 : {0} \n", this.Focal_Length);
                Console
                    .WriteLine("ViewAngleH : {0}  ViewAngleV: {1}  \n",
                    this.ViewAngleH,
                    this.ViewAngleV);
                //Console.WriteLine("H_AOV : {0}   V_AOV : {1} \n",  this.H_AOV, this.V_AOV);
            }
        }
    }
}
