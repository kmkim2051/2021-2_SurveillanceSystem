using System;
using System.Collections.Generic;
using System.Text;

namespace surveillance_system
{
   public partial class Program
    {
        public class Road
        {
            double[] laneVector;

            // 가로 도로 좌표
            // 첫번째 인덱스 = 도로 번호,  두번째 인덱스 = y 값
            double[,] lane_h; // 가로 - 중앙선 y 값 
            double[,] lane_h_upper; // 가로 - 중앙선 위 라인 y값 
            double[,] lane_h_lower; // 가로 - 중앙선 아래 라인 y값 

            // 세로 도로 좌표
            // 첫번째 인덱스 = 도로 번호,  두번째 인덱스 = x 값
            double[,] lane_v; // 세로 - 중앙선 x값
            double[,] lane_v_left; // 세로- 중앙선 왼쪽 라인 x값
            double[,] lane_v_right; // 세로 중앙선 오른쪽 라인 x값

            double[,] DST; // 도로 교차점

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
                double incr = 0.01;
                int laneSize = (int)((intvl + wd) * (n_interval - 1) / incr);
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
                    lane_h[i, 1] = i * (intvl + wd);
                    lane_h_upper[i, 1] = lane_h[i, 1] + wd / 2;
                    lane_h_lower[i, 1] = lane_h[i, 1] - wd / 2;
                }

                // 세로 도로 좌표 설정
                lane_v = new double[n_interval, 1];
                lane_v_left = new double[n_interval, 1];
                lane_v_right = new double[n_interval, 1];
                for (int i = 0; i < n_interval; i++)
                {
                    lane_v[i, 1] = i * (intvl + wd);
                    lane_v_left[i, 1] = lane_h[i, 1] - wd / 2;
                    lane_v_right[i, 1] = lane_h[i, 1] + wd / 2;
                }
            }
        }
    }
}
