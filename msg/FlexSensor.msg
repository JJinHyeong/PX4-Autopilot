uint64 timestamp        # time since system start (microseconds)

# ---- 각도 데이터 (1/32 deg/LSB 스케일 적용) ----
float32 axis1_deg       # 굽힘각 Axis1 (단위: deg, 범위: -105 ~ +105)
float32 axis2_deg       # 굽힘각 Axis2 수직 평면 (단위: deg, 범위: -105 ~ +105)
float32 total_deg       # 합성 굽힘각 = sqrt(axis1^2 + axis2^2)
float32 direction_deg   # 굽힘 방향 = atan2(axis2, axis1) (단위: deg)

# ---- Raw 카운트값 (스케일 적용 전 센서 원시값) ----
# raw = angle_deg * 32.0  (예: 90도 → raw = 2880)
int16 raw_axis1         # Axis1 원시 int16 카운트
int16 raw_axis2         # Axis2 원시 int16 카운트

