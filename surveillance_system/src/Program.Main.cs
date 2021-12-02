using System.Runtime.CompilerServices;
using System;
using System.Linq;

namespace surveillance_system
{
    public partial class Program
    {
        public static CCTV[] cctvs;
        public static Pedestrian[] peds;

        static void Main(string[] args)
        {
            Road road = new Road();
            /*------------------------------------------------------------------------
              % note 1) To avoid confusing, all input parameters for a distance has a unit as a milimeter
            -------------------------------------------------------------------------*/
            // Configuration: surveillance cameras
            // constant
            Random rand = new Random();
            const double Lens_FocalLength = 2.8; // mm, [2.8 3.6 6 8 12 16 25]
            const double WD = 3.6; // (mm) width, horizontal size of camera sensor
            const double HE = 2.7; // (mm) height, vertical size of camera sensor

            // const double Diag = Math.Sqrt(WD*WD + HE*HE), diagonal size
            const double imW = 1920; // (pixels) image width
            const double imH = 1080; // (pixels) image height

            // Installation [line_23]
            const double Angle_H = 0; // pi/2, (deg), Viewing Angle (Horizontal Aspects)
            const double Angle_V = 0; // pi/2, (deg), Viewing Angle (Vertical Aspects)

            // configuration: road
            const int Road_WD = 5000;
            bool On_Road_Builder = true; // 0:No road, 1:Grid

            int Road_Width = 0;
            int Road_Interval = 0;
            int Road_N_Interval = 0;
            if (On_Road_Builder)
            {
                Road_Width = 2000; // mm
                Road_Interval = 10000; // mm, 10 meter
                Road_N_Interval = 3;
            }

            bool Opt_Observation = false;
            bool Opt_Demo = false;
            int[] log_PED_position = null;
            if (Opt_Demo)
            {
                log_PED_position = new int[5];
            }

            // Step 1-2) calculate vertical/horizontal AOV
            double H_AOV = RadToDeg(2 * Math.Atan(WD / (2 * Lens_FocalLength))); // Horizontal AOV
            double V_AOV = RadToDeg(2 * Math.Atan(HE / (2 * Lens_FocalLength))); // Vertical AOV

            // double D_AOV = RadToDeg(2 * Math.Atan(Diag / (2 * Lens_FocalLength)));
            // (mm) distance
            int[] X = new int[100000];
            double[] H_FOV_ = new double[100000];
            double[] V_FOV_ = new double[100000];

            for (int i = 0; i < 100000; i++)
            {
                X[i] = i + 1;
                H_FOV_[i] = X[i] * WD / Lens_FocalLength;
                V_FOV_[i] = X[i] * HE / Lens_FocalLength;
            }

            // [line_55]----------------------------------------------------------------------
            const int N_CCTV = 6;
            double[] Dist = new double[10000];
            double[] Height = new double[10000];
            for (int i = 0; i < 10000; i++)
            {
                Dist[i] = i;
                Height[i] = i;
            }

            // Configuration: Pedestrian (Target Object)
            const int N_Ped = 5;
            const int Ped_Width = 900; // (mm)
            const int Ped_Height = 1700; // (mm)
            const int Ped_Velocity = 1500; // (mm/s)

            // D1 = 90
            // D2 = 270
            // Configuration: simulation time
            const double aUnitTime = 100 * 0.001;

            cctvs = new CCTV[N_CCTV];
            for (int i = 0; i < N_CCTV; i++)
            {
                cctvs[i] = new CCTV();
            }
            peds = new Pedestrian[N_Ped];
            for (int i = 0; i < N_Ped; i++)
            {
                peds[i] = new Pedestrian();
            }

            if (On_Road_Builder)
            {
                road.roadBuilder(Road_Width, Road_Interval, Road_N_Interval, N_CCTV, N_Ped);
                road.printRoadInfo();

                for (int i = 0; i < N_Ped; i++)
                {
                    Console.WriteLine("{0}번째 보행자 = ({1}, {2}) ", i + 1, peds[i].X, peds[i].Y);
                }
                Console.WriteLine("\n============================================================\n");
                for (int i = 0; i < N_CCTV; i++)
                {
                    Console.WriteLine("{0}번째 cctv = ({1}, {2}) ", i + 1, cctvs[i].X, cctvs[i].Y);
                }

                //ped
                foreach(Pedestrian ped in peds)
                {
                    double minDist = 0.0;
                    int idx_minDist = 0;
                    double[] Dist_Map = new double[road.DST.GetLength(0)];
                    
                    Calc_Dist_and_get_MinDist(road.DST, ped.X, ped.Y, ref Dist_Map, ref minDist, ref idx_minDist);
                    double dst_x = road.DST[idx_minDist, 0];
                    double dst_y = road.DST[idx_minDist, 1];

                    Console.WriteLine("\n============================================================\n");
                    Console.WriteLine("{0}번째 보행자 -  {1}번째 목적지(좌표: {2}, {3}) ",
                        Array.IndexOf(peds, ped), idx_minDist, dst_x, dst_y);

                    // 보행자~목적지 벡터
                    double[] A = new double[2];
                    A[0] = dst_x - ped.X;
                    A[1] = dst_y - ped.Y;

                    double[] B = { 0.001, 0.001 };
                    double direction = Math.Round(Math.Acos(InnerProduct(A, B) / (Norm(A) * Norm(B))),2);
                    ped.define_PED(Ped_Width, Ped_Height, direction, dst_x, dst_y, Ped_Velocity);
                    ped.TTL = (int)Math.Ceiling((minDist / ped.Velocity) / aUnitTime);
                    ped.printPedInfo();
                }


                Console.WriteLine("\n============================================================\n\n\n");

                // cctv 
                for (int i = 0; i < N_CCTV; i++)
                {


                    cctvs[i].Z =
                        (int)Math.Ceiling(rand.NextDouble() * (Height.Max() - 3000)) + 3000; // milimeter
                    cctvs[i].WD = WD;
                    cctvs[i].HE = HE;
                    cctvs[i].imW = (int)imW;
                    cctvs[i].imH = (int)imH;
                    cctvs[i].Focal_Length = Lens_FocalLength;
                    cctvs[i].ViewAngleH = rand.NextDouble() * 360;
                    cctvs[i].ViewAngleV = -35 - 20 * rand.NextDouble();
                    cctvs[i].H_AOV = 2 * Math.Atan(WD / (2 * Lens_FocalLength));
                    cctvs[i].V_AOV = 2 * Math.Atan(WD / (2 * Lens_FocalLength));

                    // Line 118~146
                    /*  여기부턴 Road_Builder 관련 정보가 없으면 의미가 없을거같아서 주석처리했어용..
                        그리고 get_Sectoral_Coverage 이런함수도 지금은 구현해야할지 애매해서..?

                    [CCTV(i).R_blind, CCTV(i).R_eff, BorderLine_blind, BorderLine_eff] = get_Sectoral_Coverage(CCTV(i).H_AOV, CCTV(i).V_AOV, ...
                                                                                                   CCTV(i).ViewAngleH, CCTV(i).ViewAngleV, ...
                                                                                                   Ped_Height, CCTV(i).X, CCTV(i).Y, CCTV(i).Z);
                    CCTV(i).BorderLine_blind_X(1,:) = BorderLine_blind(:,1);
                    CCTV(i).BorderLine_blind_Y(1,:) = BorderLine_blind(:,2);
                    
                    CCTV(i).BorderLine_eff_X(1,:) = BorderLine_eff(:,1);
                    CCTV(i).BorderLine_eff_Y(1,:) = BorderLine_eff(:,2);

                    */
                    /*
                     [CCTV(i).PPM_H(1,:), CCTV(i).PPM_V(1,:), CCTV(i).SurvDist_H(1,:), CCTV(i).SurvDist_V(1,:)] ...
                    = get_PixelDensity(Dist, CCTV(i).WD, CCTV(i).HE, CCTV(i).Focal_Length, CCTV(i).imW, CCTV(i).imH);
                    Eff_Dist_Range = CCTV(i).R_blind:CCTV(i).R_eff;
                    */
                    /* =================================
                     *  추가 line 460 (깃허브 127~128)
                     *  밑에 라인 495~544에서 필요한 변수를 여기서 처리해서 일단 얘는 옮겨놨습니다!
                     * =================================*/
                    cctvs[i]
                        .get_PixelDensity(Dist,
                        cctvs[i].WD,
                        cctvs[i].HE,
                        cctvs[i].Focal_Length,
                        cctvs[i].imW,
                        cctvs[i].imH);

                    /*
                    [FOV_X2, FOV_Y2] = get_V_FOV(Dist, CCTV(i).HE, CCTV(i).Focal_Length, CCTV(i).ViewAngleV, CCTV(i).X, CCTV(i).Z);
                    CCTV(i).V_FOV_X0(1,:) = FOV_X2(1,:);
                    CCTV(i).V_FOV_X1(1,:) = FOV_X2(2,:);
                    CCTV(i).V_FOV_X2(1,:) = FOV_X2(3,:);
                    CCTV(i).V_FOV_Z0(1,:) = FOV_Y2(1,:);
                    CCTV(i).V_FOV_Z1(1,:) = FOV_Y2(2,:);
                    CCTV(i).V_FOV_Z2(1,:) = FOV_Y2(3,:);    
            %         CCTV.EffectDist(i,:) = EffectDist;
                            
                    [FOV_X, FOV_Y] = get_H_FOV(Eff_Dist_Range, CCTV(i).WD, CCTV(i).Focal_Length,  CCTV(i).ViewAngleH, CCTV(i).X, CCTV(i).Y, CCTV(i).SurvDist_H(1,:));
                    CCTV(i).H_FOV_X0(1,:) = FOV_X(1,:);
                    CCTV(i).H_FOV_X1(1,:) = FOV_X(2,:);
                    CCTV(i).H_FOV_X2(1,:) = FOV_X(3,:);
                    CCTV(i).H_FOV_Y0(1,:) = FOV_Y(1,:);
                    CCTV(i).H_FOV_Y1(1,:) = FOV_Y(2,:);
                    CCTV(i).H_FOV_Y2(1,:) = FOV_Y(3,:);
                  */

                    cctvs[i].printCCTVInfo();
                }
            }

            // 추가 % Initialize 8 maps (깃허브 line 158~)
            /*
            double[,] Dist_MAP_H1 = new double[N_CCTV, N_Ped];
            double[,] Dist_MAP_H2 = new double[N_CCTV, N_Ped];
            double[,] Dist_MAP_V1 = new double[N_CCTV, N_Ped];
            double[,] Dist_MAP_V2 = new double[N_CCTV, N_Ped];
            double[,] SurvDist_MAP_H1 = new double[N_CCTV, N_Ped];
            double[,] SurvDist_MAP_H2 = new double[N_CCTV, N_Ped];
            double[,] SurvDist_MAP_V1 = new double[N_CCTV, N_Ped];
            double[,] SurvDist_MAP_V2 = new double[N_CCTV, N_Ped];

            //  추가 (깃허브 line 168~207)
            for (int i = 0; i < N_CCTV; i++)
            {
                for (int j = 0; j < N_Ped; j++)
                {
                    Dist_MAP_H1[i, j] =
                        Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H1[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H1[1], 2));
                    Dist_MAP_H2[i, j] =
                        Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H2[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H2[1], 2));
                    Dist_MAP_V1[i, j] =
                        Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V1[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_V1[1], 2));
                    Dist_MAP_V1[i, j] =
                        Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V2[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_V2[1], 2));

                    for (int k = 0; k < cctvs[i].SurvDist_H.Length; k++)
                    {
                        if (cctvs[i].SurvDist_H[k] >= Dist_MAP_H1[i, j])
                        {
                            SurvDist_MAP_H1[i, j] = k;
                            break;
                        }
                    }

                    for (int k = 0; k < cctvs[i].SurvDist_H.Length; k++)
                    {
                        if (cctvs[i].SurvDist_H[k] >= Dist_MAP_H2[i, j])
                        {
                            SurvDist_MAP_H2[i, j] = k;
                            break;
                        }
                    }

                    for (int k = 0; k < cctvs[i].SurvDist_V.Length; k++)
                    {
                        if (cctvs[i].SurvDist_V[k] >= Dist_MAP_V1[i, j])
                        {
                            SurvDist_MAP_V1[i, j] = k;
                            break;
                        }
                    }

                    for (int k = 0; k < cctvs[i].SurvDist_V.Length; k++)
                    {
                        if (cctvs[i].SurvDist_V[k] >= Dist_MAP_V2[i, j])
                        {
                            SurvDist_MAP_V2[i, j] = k;
                            break;
                        }
                    }
                }
            }
            */


            // 거리 검사
            int[,] candidate_detected_ped_h = new int[N_CCTV, N_Ped];
            int[,] candidate_detected_ped_v = new int[N_CCTV, N_Ped];

            for(int i = 0; i < N_CCTV; i++)
            {
                for(int j = 0; j < N_Ped; j++)
                {
                    double dist_h1 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H1[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H1[1], 2));
                    double dist_h2 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H2[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H2[1], 2));
                    double dist_v1 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V1[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_V1[1], 2));
                    double dist_v2 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V2[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_V2[1], 2));

                    foreach(double survdist_h in cctvs[i].SurvDist_H)
                    {
                        if (dist_h1 <=survdist_h  && dist_h2 <= survdist_h)
                        {
                            candidate_detected_ped_h[i, j] = 1;
                        }
                    }

                    foreach (double survdist_v in cctvs[i].SurvDist_V)
                    {
                        if (dist_h1 <= survdist_v && dist_h2 <= survdist_v)
                        {
                            candidate_detected_ped_v[i, j] = 1;
                        }
                    }
                }
            }
            

        }
    }
}
