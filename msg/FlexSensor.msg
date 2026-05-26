uint64 timestamp        # time since system start (microseconds)
uint32 device_id        # ADS1115 device ID (from adc_report)

int32[4]   raw_adc          # raw 16-bit ADC values, AIN0..AIN3
float32[4] voltage_v        # measured voltage at each channel (V)
float32[4] bend_pct         # bend percentage: 0.0=flat, 100.0=fully bent (%)
float32[4] flex_angle_deg   # estimated flex angle: 0=flat, 90=fully bent (degrees)
