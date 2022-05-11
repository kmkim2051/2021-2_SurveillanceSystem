// using Internal;
using System.Runtime.CompilerServices;
using System;
using System.Linq;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

namespace surveillance_system
{
    public partial class Program
    {
        public static CCTV[] cctvs;
        public static Pedestrian[] peds;

        // Configuration: simulation time
        const double aUnitTime = 100 * 0.001; // (sec)
        public static Road road = new Road();

        /* --------------------------------------
         * 추적 여부 검사 함수
        -------------------------------------- */
        static int[] checkDetection(int N_CCTV, int N_Ped)
        {

            int[] returnArr = new int[N_Ped]; // 반환할 탐지 결과 (1: 탐지  0: 거리상 미탐지  -1: 방향 미스)

            // 거리 검사
            int[,] candidate_detected_ped_h = new int[N_CCTV, N_Ped];
            int[,] candidate_detected_ped_v = new int[N_CCTV, N_Ped];

            for (int i = 0; i < N_CCTV; i++)
            {
                
                for (int j = 0; j < N_Ped; j++)
                {
                    double dist_h1 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H1[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H1[1], 2));
                    double dist_h2 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_H2[0], 2) +
                            Math.Pow(cctvs[i].Y - peds[j].Pos_H2[1], 2));
                    double dist_v1 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V1[0], 2) +
                            Math.Pow(cctvs[i].Z - peds[j].Pos_V1[1], 2));
                    double dist_v2 = Math
                            .Sqrt(Math.Pow(cctvs[i].X - peds[j].Pos_V2[0], 2) +
                            Math.Pow(cctvs[i].Z - peds[j].Pos_V2[1], 2)) ;

                    foreach (double survdist_h in cctvs[i].SurvDist_H)
                    {
                        if (dist_h1 <= survdist_h*100*10 && dist_h2 <= survdist_h * 100 * 10)
                        {
                            candidate_detected_ped_h[i, j] = 1;
                        }
                    }
                    foreach (double survdist_v in cctvs[i].SurvDist_V)
                    {
                        if (dist_v1 <= survdist_v * 100 * 10 && dist_v2 <= survdist_v * 100 * 10)
                        {
                            candidate_detected_ped_v[i, j] = 1;
                        }
                    }

                    // if (cctvs[i].isPedInEffDist(peds[j])) {
                    //   candidate_detected_ped_h[i, j] = 1;
                    //   candidate_detected_ped_v[i, j] = 1;
                    // }

                      // candidate_detected_ped_h[i, j] = 1;
                      // candidate_detected_ped_v[i, j] = 1;
                }
            }



            // return returnArr;

            // 각 CCTV의 보행자 탐지횟수 계산
            int[] cctv_detecting_cnt = new int[N_CCTV];
            int[] cctv_missing_cnt = new int[N_CCTV];

            int[,] missed_map_h = new int[N_CCTV, N_Ped];
            int[,] missed_map_v = new int[N_CCTV, N_Ped];

            int[,] detected_map = new int[N_CCTV, N_Ped];

            // 각도 검사 
            for (int i = 0; i < N_CCTV; i++)
            {
                double cosine_H_AOV = Math.Cos(cctvs[i].H_AOV / 2);
                double cosine_V_AOV = Math.Cos(cctvs[i].V_AOV / 2);

                for (int j = 0; j < N_Ped; j++)
                {

                    // 거리상 미탐지면 넘어감 
                    if (candidate_detected_ped_h[i, j] != 1 || candidate_detected_ped_v[i, j] != 1)
                    {                      
                        continue;
                    }
                    
                    int h_detected = -1;
                    int v_detected = -1;

                    // 거리가 범위 내이면
                    if (candidate_detected_ped_h[i, j] == 1)
                    {
                        // len equals Dist
                        int len = cctvs[i].H_FOV.X0.GetLength(0);
                        double[] A = { cctvs[i].H_FOV.X0[len - 1] - cctvs[i].X, cctvs[i].H_FOV.Y0[len - 1] - cctvs[i].Y };
                        double[] B = { peds[j].Pos_H1[0] - cctvs[i].X, peds[j].Pos_H1[1] - cctvs[i].Y };
                        double cosine_PED_h1 = InnerProduct(A, B) / (Norm(A) * Norm(B));

                        B[0] = peds[j].Pos_H2[0] - cctvs[i].X;
                        B[1] = peds[j].Pos_H2[1] - cctvs[i].Y;
                        double cosine_PED_h2 = InnerProduct(A, B) / (Norm(A) * Norm(B));

                        // horizontal 각도 검사 
                        if (cosine_PED_h1 >= cosine_H_AOV && cosine_PED_h2 >= cosine_H_AOV)
                        {
                            //감지 됨
                            h_detected = 1;
                        }
                        else
                        {
                            h_detected = 0;
                        }
                    }

                    // vertical  각도 검사 
                    if (candidate_detected_ped_v[i, j] == 1)
                    {
                      // Surv_SYS_v210202.m [line 260]
                      /*         
                        if ismember(j, Candidates_Detected_PED_V1)
                        A = [CCTV(i).V_FOV_X0(1,:); CCTV(i).V_FOV_Z0(1,:)] - [CCTV(i).X; CCTV(i).Z];
                        B = [PED(j).Pos_V1(1); PED(j).Pos_V1(2)] - [CCTV(i).X; CCTV(i).Z]; 
                      */
                        int len = cctvs[i].V_FOV.X0.GetLength(0);
                        double[] A = { cctvs[i].V_FOV.X0[len - 1] - cctvs[i].X, cctvs[i].V_FOV.Z0[len - 1] - cctvs[i].Z };
                        double[] B = { peds[j].Pos_V1[0] - cctvs[i].X, peds[j].Pos_V1[1] - cctvs[i].Z };
                        double cosine_PED_v1 = InnerProduct(A, B) / (Norm(A) * Norm(B));

                        B[0] = peds[j].Pos_V2[0] - cctvs[i].X;
                        B[1] = peds[j].Pos_V2[1] - cctvs[i].Z;
                        double cosine_PED_v2 = InnerProduct(A, B) / (Norm(A) * Norm(B));

                        if (cosine_PED_v1 >= cosine_V_AOV && cosine_PED_v2 >= cosine_V_AOV)
                        {
                            //감지 됨
                            v_detected = 1;
                        }
                        else
                        {
                            v_detected = 0;
                        }
                    }

                  
                    if (h_detected == 1 && v_detected == 1)
                    {
                        detected_map[i, j] = 1;
                        // 각 CCTV[i]의 보행자 탐지 횟수 증가
                        cctv_detecting_cnt[i]++;

                        returnArr[j] = 1;
                        // 220407
                        cctvs[i].detectedPedIndex.Add(j);
                    }
                    // 방향 미스 (h or v 중 하나라도 방향이 맞지 않는 경우)
                    else // cctv[i]가 보행자[j]를 h or v 탐지 실패 여부 추가
                    {
                        cctv_missing_cnt[i]++;
                        
                        if(h_detected == 0) missed_map_h[i, j] = 1;

                        if(v_detected == 0) missed_map_v[i, j] = 1;

                        returnArr[j] = (returnArr[j] == 1 ? 1 : -1);

                        /*
                        if(h_detected != 1)
                        {
                            Console.WriteLine("[{0}] horizontal 감지 못함", h_detected);
                        }
                        else if(v_detected != 1)
                        {
                            Console.WriteLine("[{0}] vertical 감지 못함 ", v_detected);
                        }
                        */
                    }


                } // 탐지 여부 계산 완료
            }



            // 여기부턴 h or v 각각 분석
            // 각 cctv는 h, v 축에서 얼마나 많이 놓쳤나?
            int[] cctv_missing_count_h = new int[N_CCTV];
            int[] cctv_missing_count_v = new int[N_CCTV];

            for(int i = 0 ; i < N_CCTV; i++)
            for(int j = 0 ; j < N_Ped; j++)
            {
                cctv_missing_count_h[i] += missed_map_h[i, j];
                cctv_missing_count_v[i] += missed_map_v[i, j];
            }
            // 보행자를 탐지한 cctv 수
            int[] detecting_cctv_cnt = new int[N_Ped];
            // 보행자를 탐지하지 못한 cctv 수
            int[] missing_cctv_cnt = new int[N_Ped];

            //Console.WriteLine("=== 성공 ====");
            // detection 결과 출력 
            for (int i = 0; i < N_CCTV; i++)
            {
                for (int j = 0; j < N_Ped; j++)
                {
                    if (detected_map[i, j] == 1)
                    {
                        detecting_cctv_cnt[j]++;
                    }
                    else
                    {
                        missing_cctv_cnt[j]++;
                    }
                }
            }

// for time check
            // Console.WriteLine("---------------------------------");
            // Console.WriteLine("   성공  ||   실패  ");
            // for (int i = 0; i < N_Ped; i++)
            // {
            //     if (detecting_cctv_cnt[i] == 0)
            //     {
            //         Console.WriteLine("         ||   ped{0} ", i + 1);
            //     }
            //     else
            //     {
            //         Console.WriteLine("ped{0} ", i + 1);
            //     }
            // }
            // Console.WriteLine("---------------------------------");


            return returnArr;
        }
        
        static void Main(string[] args)
        {
            /*------------------------------------------------------------------------
              % note 1) To avoid confusing, all input parameters for a distance has a unit as a milimeter
            -------------------------------------------------------------------------*/
            // Configuration: surveillance cameras
            // constant
            const int N_CCTV =36;
            const int N_Ped = 10;

            Random rand = new Random();
            const double Lens_FocalLength = 2.8; // mm, [2.8 3.6 6 8 12 16 25]
            const double WD = 3.6; // (mm) width, horizontal size of camera sensor
            const double HE = 2.7; // (mm) height, vertical size of camera sensor

            // const double Diag = Math.Sqrt(WD*WD + HE*HE), diagonal size
            const double imW = 1920; // (pixels) image width
            const double imH = 1080; // (pixels) image height

            const double cctv_rotate_degree = 90; // 30초에 한바퀴
            // Installation [line_23]
            const double Angle_H = 0; // pi/2, (deg), Viewing Angle (Horizontal Aspects)
            const double Angle_V = 0; // pi/2, (deg), Viewing Angle (Vertical Aspects)

            // configuration: road
            const int Road_WD = 5000; // 이거 안쓰는 변수? Road_Width 존재
            bool On_Road_Builder = true; // 0:No road, 1:Grid
          
            int Road_Width = 0;
            int Road_Interval = 0;
            int Road_N_Interval = 0;
            if (On_Road_Builder)
            {
                Road_Width = 10000; // mm
                Road_Interval = 88000; // mm, 10 meter
                Road_N_Interval =5;
            }

            bool Opt_Observation = false;
            bool Opt_Demo = false;
            int[] log_PED_position = null;
            if (Opt_Demo)
            {
                log_PED_position = new int[5];
            }
            // time check start
            // double accTime = 0.0;

            // ped csv file 출력 여부
            bool createPedCSV = false;

            double rotateTerm = 30.0; // sec

            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();

            // Step 1-2) calculate vertical/horizontal AOV
            double H_AOV = RadToDeg(2 * Math.Atan(WD / (2 * Lens_FocalLength))); // Horizontal AOV
            double V_AOV = RadToDeg(2 * Math.Atan(HE / (2 * Lens_FocalLength))); // Vertical AOV

            // double D_AOV = RadToDeg(2 * Math.Atan(Diag / (2 * Lens_FocalLength)));
            // (mm) distance
            double[] Dist = new double[10000];
            int dist_len = 100000;
            double[] Height = new double[10000];
            for (int i = 0; i < 10000; i++)
            {
                Dist[i] = i;
                Height[i] = i;
            }

            // Configuration: Pedestrian (Target Object)
            const int Ped_Width = 900; // (mm)
            const int Ped_Height = 1700; // (mm)
            const int Ped_Velocity = 1500; // (mm/s)

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


            /* -------------------------------------------
            *  도로 정보 생성 + 보행자/CCTV 초기화 시작
            ------------------------------------------- */
            // time check


            if (On_Road_Builder)
            {
                // 도로 정보 생성, 보행자 정보 생성
                road.roadBuilder(Road_Width, Road_Interval, Road_N_Interval, N_CCTV, N_Ped);

                /*
                // debug 220428
                for(int i = 0 ; i < N_CCTV; i++) {
                    Console.Write(cctvs[i].X);
                  Console.Write(", ");
                  Console.WriteLine(cctvs[i].Y);

                }
                */
                road.printRoadInfo();


                /*

                //*  보행자, cctv 초기 설정
                for (int i = 0; i < N_Ped; i++)
                {
                    Console.WriteLine("{0}번째 보행자 = ({1}, {2}) ", i + 1, peds[i].X, peds[i].Y);
                }
                Console.WriteLine("\n============================================================\n");
                for (int i = 0; i < N_CCTV; i++)
                {
                    Console.WriteLine("{0}번째 cctv = ({1}, {2}) ", i + 1, cctvs[i].X, cctvs[i].Y);
                }
                */


                //ped init
                foreach(Pedestrian ped in peds)
                {
                    double minDist = 0.0;
                    //int idx_minDist = 0;
                    //double[] Dist_Map = new double[road.DST.GetLength(0)];

                    // 맨처음 위치에서 가장 가까운 도착지를 설정 (보행자 맨처음 위치는 line 314에서 setPed()로 설정)
                    double[,] newPos = road.getPointOfAdjacentRoad(road.getIdxOfIntersection(ped.X, ped.Y));
                    double dst_x = Math.Round(newPos[0, 0]);
                    double dst_y = Math.Round(newPos[0, 1]);

                    //Calc_Dist_and_get_MinDist(road.DST, ped.X, ped.Y, ref Dist_Map, ref minDist, ref idx_minDist);
                    
                    //double dst_x = road.DST[idx_minDist, 0];
                    //double dst_y = road.DST[idx_minDist, 1];

                    // 보행자~목적지 벡터
                    /*
                    double[] A = new double[2];
                    A[0] = dst_x - ped.X;
                    A[1] = dst_y - ped.Y;        

                    double[] B = { 0.001, 0 };
                    double direction = Math.Round(Math.Acos(InnerProduct(A, B) / (Norm(A) * Norm(B))),8);
                    if(ped.Y > dst_y)
                    {
                        direction = Math.Round(2 * Math.PI - direction, 8); 
                    }
                    */
                    ped.define_PED(Ped_Width, Ped_Height, dst_x, dst_y, Ped_Velocity);
                    ped.setDirection();
                    ped.TTL = (int)Math.Ceiling((minDist / ped.Velocity) / aUnitTime);
                    ped.printPedInfo();
                }
                // cctv init
                for (int i = 0; i < N_CCTV; i++)
                {
                    // 220317
                    // Height.Max() 는 고정값 (=대충 10000)..
                    // 상수로 바꿔도 될듯??
                    // default Z는 3000
                    // 3000 ~ 10000 사이 값, 즉 7000이 변하는 값
                    // default(min) : 3000, variant : 7000 
                    // maxZ = min + variant 이런식으로?..

                    // cctvs[i].Z =
                    //     (int)Math.Ceiling(rand.NextDouble() * (Height.Max() - 3000)) + 3000; // milimeter
                    cctvs[i].setZ((int)Math.Ceiling(rand.NextDouble() * (Height.Max() - 3000)) + 3000);
                    cctvs[i].WD = WD;
                    cctvs[i].HE = HE;
                    cctvs[i].imW = (int)imW;
                    cctvs[i].imH = (int)imH;
                    cctvs[i].Focal_Length = Lens_FocalLength;
                    // 220104 초기 각도 설정
                    // cctvs[i].ViewAngleH = rand.NextDouble() * 360;
                    // cctvs[i].ViewAngleV = -35 - 20 * rand.NextDouble();
                    cctvs[i].setViewAngleH(rand.NextDouble() * 360);
                    cctvs[i].setViewAngleV(-35 - 20 * rand.NextDouble());
                    
                    cctvs[i].setFixMode(true); // default (rotate)

                    cctvs[i].H_AOV = 2 * Math.Atan(WD / (2 * Lens_FocalLength));
                    cctvs[i].V_AOV = 2 * Math.Atan(HE / (2 * Lens_FocalLength));

                    // 기기 성능상의 최대 감시거리 (임시값)
                    cctvs[i].Max_Dist = 50 * 100 * 10; // 50m (milimeter)
                    // cctvs[i].Max_Dist = 500 * 100 * 100; // 500m (milimeter)


                    cctvs[i].detectedPedIndex = new List<int>();

                    // Line 118~146
                    /*  여기부턴 Road_Builder 관련 정보가 없으면 의미가 없을거같아서 주석처리했어용..
                        그리고 get_Sectoral_Coverage 이런함수도 지금은 구현해야할지 애매해서..?
                    */

                    cctvs[i]
                        .get_PixelDensity(Dist,
                        cctvs[i].WD,
                        cctvs[i].HE,
                        cctvs[i].Focal_Length,
                        cctvs[i].imW,
                        cctvs[i].imH);

                    cctvs[i].get_H_FOV(Dist, cctvs[i].WD, cctvs[i].Focal_Length, cctvs[i].ViewAngleH, cctvs[i].X, cctvs[i].Y);
                    cctvs[i].get_V_FOV(Dist, cctvs[i].HE, cctvs[i].Focal_Length, cctvs[i].ViewAngleV, cctvs[i].X, cctvs[i].Z);
                    // cctvs[i].printCCTVInfo();
                }
            }
            /* -------------------------------------------
            *  도로 정보 생성 + 보행자/CCTV 초기화 끝
            ------------------------------------------- */

            double Sim_Time = 600;
            double Now = 0;

            // Console.WriteLine(">>> Simulating . . . \n");
            int[] R_Surv_Time = new int[N_Ped]; // 탐지 
            int[] directionError = new int[N_Ped]; // 방향 미스
            int[] outOfRange = new int[N_Ped]; // 거리 범위 밖

            string[] traffic_x = new string[(int)(Sim_Time / aUnitTime)]; // csv 파일 출력 위한 보행자별 x좌표
            string[] traffic_y = new string[(int)(Sim_Time / aUnitTime)]; // csv 파일 출력 위한 보행자별 y좌표
            string[] detection = new string[(int)(Sim_Time / aUnitTime)]; // csv 파일 출력 위한 추적여부
            string header = "";

            int road_min = 0;
            int road_max = road.mapSize;

            // Console.WriteLine("simulatioin start: ");
            // simulation
            while (Now < Sim_Time)
            {
                //Console.WriteLine(".");
                // 추적 검사
                int[] res = checkDetection(N_CCTV, N_Ped);
                // threading.. error
                // int[] res = new int[N_Ped];

                // Thread ThreadForWork = new Thread( () => { res = checkDetection(N_CCTV, N_Ped); });     
                // ThreadForWork.Start();
                
                for(int i = 0; i < res.Length; i++)
                {
                    detection[i] += Convert.ToString(res[i]) + ",";

                    if (res[i] == 0) outOfRange[i]++;
                    else if (res[i] == -1) directionError[i]++;
                    else if (res[i] == 1) R_Surv_Time[i]++;
                }

                /* 220407 
                 * 보행자 방향 따라 CCTV 회전 제어
                 * 각 보행자가 탐지/미탐지 여부를 넘어서
                 * 특정 CCTV가 지금 탐지한 보행자의 정보를 알아야함
                 * 그래야 보행자의 범위 내 위치, 방향을 읽어서
                 * 보행자의 이동 방향으로 CCTV 회전 여부, 회전 시 방향 및 각도 설정 가능
                */

                // 이동
                for (int i = 0; i < peds.Length; i++)
                {
                    if(peds[i].X < road_min || peds[i].X > road_max)
                    {
                        traffic_x[i] += "Out of range,";
                    }
                    else
                    {
                        traffic_x[i] += Math.Round(peds[i].X, 2) + ",";
                    }

                    if (peds[i].Y < road_min || peds[i].Y > road_max)
                    {
                        traffic_y[i] += "Out of range,";
                    }
                    else
                    {
                        traffic_y[i] += Math.Round(peds[i].Y, 2) + ",";
                    }

                    peds[i].move();
                }
                
                // 220317 cctv rotation
                for(int i = 0 ; i < N_CCTV; i++)
                {
                    // 220331 rotate 후 fov 재계산
                    // 30초마다 한바퀴 돌도록 -> 7.5초마다 90도
                    // Now는 현재 simulation 수행 경과 시간
                    // 360/cctv_rotate_degree = 4
                    // 30/4 = 7.5
                    if (Math.Round(Now, 2) % Math.Round(rotateTerm/(360.0/cctv_rotate_degree), 2) == 0) 
                    {
                      // cctv.setFixMode(false)로 설정해줘야함!
                      // Console.WriteLine("[Rotate] Now: {0}, Degree: {1}", Math.Round(Now, 2), cctvs[i].ViewAngleH);
                      cctvs[i].rotateHorizon(cctv_rotate_degree); // 90
                      // 회전후 수평 FOV update (지금은 전부 Update -> 시간 오래걸림 -> 일부만(일부FOV구성좌표만)해야할듯)
                      cctvs[i].get_H_FOV(Dist, cctvs[i].WD, cctvs[i].Focal_Length, cctvs[i].ViewAngleH, cctvs[i].X, cctvs[i].Y);
                    }
                }
                

                header += Convert.ToString(Math.Round(Now,1))+",";
                Now += aUnitTime;
            }
            stopwatch.Stop();

            // // create .csv file
            if (createPedCSV) 
            {
                for (int i = 0; i < peds.Length; i++)
                {
                    string fileName = "ped"+i+".csv";
                    using (System.IO.StreamWriter file = new System.IO.StreamWriter(@fileName))
                    {
                        file.WriteLine(header);
                        file.WriteLine(traffic_x[i]);
                        file.WriteLine(traffic_y[i]);
                        file.WriteLine(detection[i]);
                    }
                }
            }
            double totalSimCount = Sim_Time / aUnitTime * N_Ped;

            // 결과(탐지율)
            Console.WriteLine("====== Surveillance Time Result ======");
            Console.WriteLine("N_CCTV: {0}, N_Ped: {1}", N_CCTV, N_Ped);
            Console.WriteLine("[Result]");
            Console.WriteLine("  - Execution time : {0}", stopwatch.ElapsedMilliseconds + "ms");
            Console.WriteLine("[Fail]");
            Console.WriteLine("  - Out of Range: {0:F2}% ({1}/{2})", 100*outOfRange.Sum()/totalSimCount, outOfRange.Sum(), totalSimCount);
            Console.WriteLine("  - Direction Error: {0:F2}% ({1}/{2})", 100*directionError.Sum()/totalSimCount, directionError.Sum(), totalSimCount);
            Console.WriteLine("[Success]");
            Console.WriteLine("  - Surveillance Time: {0:F2}% ({1}/{2})\n", 100*R_Surv_Time.Sum()/totalSimCount, R_Surv_Time.Sum(), totalSimCount);

            // 결과(시간)
            // Console.WriteLine("Execution time : {0}", stopwatch.ElapsedMilliseconds + "ms");
            // accTime += stopwatch.ElapsedMilliseconds;
            
            // Console.WriteLine("\n============ RESULT ============");
            // Console.WriteLine("CCTV: {0}, Ped: {1}", N_CCTV, N_Ped);
            // Console.WriteLine("Execution time : {0}\n", (accTime / 1000.0 ) + " sec");
        }
    }
}
