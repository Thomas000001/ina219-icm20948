/*arduino驅動代碼改進後的icm20948驅動代碼，讀取icm20948數值*/
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/i2c.h>
// #include <stdio.h>

// // 包含您的 ICM20948 驅動程式標頭檔
// #include "icm20948_zephyr.h"

// // 使用您提供的裝置樹節點標籤
// #define I2C_SENSOR_NODE DT_NODELABEL(icm20948) // <--- 使用您指定的節點標籤

// #if !DT_NODE_HAS_STATUS(I2C_SENSOR_NODE, okay)
// #error "Unsupported board: icm20948 devicetree node not found or not okay"
// // 為了讓程式碼能編譯，但執行時會出錯 (如果上面 #error 未中止編譯)
// static const struct i2c_dt_spec dev_icm20948 = {0};
// #else
// // 使用 I2C_DT_SPEC_GET 獲取設備規格
// static const struct i2c_dt_spec dev_icm20948 = I2C_DT_SPEC_GET(I2C_SENSOR_NODE);
// #endif

// // 宣告 ICM20948 裝置結構實例
// static icm20948_dev_t my_icm_sensor;

// int main(void) {
//     int ret;

//     printf("nRF5340DK ICM20948 Magnetometer X-axis Test (using I2C_DT_SPEC_GET and printf)\n");

//     // 檢查 I2C 設備是否準備就緒
//     // dev_icm20948.bus 是指向 I2C 控制器設備的指標
//     if (dev_icm20948.bus == NULL || !device_is_ready(dev_icm20948.bus)) {
//         printf("Error: I2C bus for ICM20948 not found or not ready!\n");
//         return -ENODEV;
//     }
//     printf("I2C bus for ICM20948: %s, ready.\n", dev_icm20948.bus->name);
//     printf("ICM20948 I2C address from DT_SPEC: 0x%X\n", dev_icm20948.addr);

//     // 初始化 ICM20948 感測器
//     // 將 dev_icm20948.bus 和 dev_icm20948.addr 傳遞給驅動程式的 init 函式
//     // 注意：我們的 icm20948_init 函式第二個參數是 bus device, 第四個是 address
//     ret = icm20948_init(&my_icm_sensor, dev_icm20948.bus, false, dev_icm20948.addr);
//     if (ret != 0) {
//         printf("Error: Failed to initialize ICM20948: error %d\n", ret);
//         return ret;
//     }
//     printf("ICM20948 core initialized successfully.\n");

//     // 檢查 WhoAmI (可選)
//     uint8_t whoami_val;
//     ret = icm20948_who_am_i(&my_icm_sensor, &whoami_val);
//     if (ret == 0) {
//         printf("ICM20948 WhoAmI: 0x%02X (Expected: 0x%02X)\n", whoami_val, ICM20948_WHO_AM_I_CONTENT);
//         if (whoami_val != ICM20948_WHO_AM_I_CONTENT) {
//             printf("Warning: WhoAmI value mismatch!\n");
//         }
//     } else {
//         printf("Error: Failed to read WhoAmI: error %d\n", ret);
//     }

//     // 初始化磁力計 (AK09916)
//     ret = icm20948_init_magnetometer(&my_icm_sensor);
//     if (ret != 0) {
//         printf("Error: Failed to initialize magnetometer (AK09916): error %d\n", ret);
//     } else {
//         printf("Magnetometer (AK09916) initialized successfully.\n");
//         uint16_t mag_id;
//         ret = icm20948_who_am_i_mag(&my_icm_sensor, &mag_id);
//         if (ret == 0) {
//             printf("AK09916 WhoAmI: 0x%04X (Expected: 0x4809 or 0x0948)\n", mag_id);
//         } else {
//             printf("Error: Failed to read magnetometer WhoAmI: %d\n", ret);
//         }
//     }

//     // 設定感測器範圍 (可選)
//     ret = icm20948_set_acc_range(&my_icm_sensor, ICM20948_ACC_RANGE_4G);
//     if (ret != 0) printf("Warning: Failed to set accelerometer range: %d\n", ret);

//     ret = icm20948_set_gyr_range(&my_icm_sensor, ICM20948_GYRO_RANGE_500);
//     if (ret != 0) printf("Warning: Failed to set gyroscope range: %d\n", ret);

//     xyzFloat_t mag_data;
//     float start = k_uptime_get();
//     while (1) {
//         ret = icm20948_read_sensor_data(&my_icm_sensor);
//         if (ret != 0) {
//             printf("Error: Failed to read sensor data: error %d\n", ret);
//             k_msleep(1000);
//             continue;
//         }

//         ret = icm20948_get_mag_values(&my_icm_sensor, &mag_data);
//         float end = k_uptime_get();
//         float duty = (end - start)/1000;
//         if (ret != 0) {
//             printf("Error: Failed to get magnetometer values: error %d\n", ret);
//         } else {
//             printf("T:%.2f\n", duty);
//             printf("M:%.2f\n", mag_data.x);
//         }
//         // fflush(stdout);
        // k_msleep(2);
//     }

//     return 0;
// }

/*arduino驅動代碼icm20948+ina219*/
// #include <zephyr/kernel.h>
// #include <zephyr/drivers/sensor.h> // For INA219
// #include <zephyr/drivers/i2c.h>    // For new ICM20948 driver's DT spec
// #include <zephyr/sys/printk.h>   // For printk (if still used by other parts)
// #include <stdio.h>               // For printf
// #include <math.h>                // For fabsf if needed for filtering/slope

// // 包含新的 ICM20948 驅動程式標頭檔
// #include "icm20948_zephyr.h" // <--- 新的驅動程式

// /* I2C 设备定义 for ICM20948 */
// #define I2C_NODE_ICM20948 DT_NODELABEL(icm20948) // 您的裝置樹節點
// static const struct i2c_dt_spec dev_icm20948_spec = I2C_DT_SPEC_GET(I2C_NODE_ICM20948);

// /* 新的 ICM20948 驅動程式實例 */
// static icm20948_dev_t my_icm_sensor; // <--- 新的驅動程式裝置結構

// /* 传感器数据刷新时间 */
// #define SENSOR_SAMPLE_INTERVAL K_MSEC(4.1)
// #define ICM_SENSOR_SAMPLE_INTERVAL K_MSEC(9) // 可以根據需要調整

// /* 全局变量 - 移除舊的 ICM20948 狀態變數 */
// // extern inv_icm20948_state st; // <--- 移除

// #define ICM20948_STACK_SIZE 1024
// #define INA219_STACK_SIZE 1024
// #define ICM20948_PRIORITY 7
// #define INA219_PRIORITY 7

// K_THREAD_STACK_DEFINE(icm20948_stack, ICM20948_STACK_SIZE);
// K_THREAD_STACK_DEFINE(ina219_stack, INA219_STACK_SIZE);
// struct k_thread icm20948_thread_data;
// struct k_thread ina219_thread_data;

// K_MUTEX_DEFINE(uart_mutex);
// K_MUTEX_DEFINE(i2c_mutex); // 保護 I2C 匯流排

// /* 移动平均窗口大小 */
// /*20250918 On the operation of the moving average, I change to shift opertaion(位移運算)*/
// #define CURRENT_FILTER_SIZE 5
// #define SLOPE_FILTER_SIZE 5      // 雖然我們只讀取X軸，但保留結構以防未來使用
// #define MAG_MOVING_AVG_SIZE 2
// // #define CURRENT_FILTER_SIZE 3       // 使用 2^3 = 8 點移動平均
// // #define SLOPE_FILTER_SIZE 2     // 使用 2^2 = 4 點移動平均
// // #define MAG_MOVING_AVG_SIZE 2   

// /* 共享数据结构 */
// struct shared_data {
//     float current;
//     float filtered_mag_x;      // <--- 修改: 儲存 X 軸磁場值
//     // float filtered_slope;   // 如果不再計算斜率，可以移除
//     float mag_sample_time;
//     float cur_sample_time;
// };

// static struct shared_data motor_data;
// static K_MUTEX_DEFINE(motor_data_mutex);

// /* 移动平均器结构体 */
// typedef struct {
//     float values[MAG_MOVING_AVG_SIZE]; // <--- 調整大小以匹配磁力計濾波需求
//     int index;
//     int count;
//     float sum;
// } MovingAverage;

// /* 数据滤波器 for Mag X */
// static MovingAverage mag_x_filter = { // <--- 重新命名以反映其用途
//     .index = 0,
//     .count = 0,
//     .sum = 0.0f
// };

// // (slope_filter 和 cycle_detector 相關的結構和變數，如果不再使用可以移除)
// // ... 如果 CycleDetector 和 slope_filter 仍然需要，請保留它們的定義 ...
// // 為了簡化，假設目前主要關注磁力計X軸和電流

// /* 初始化移动平均器 */
// static void moving_average_init(MovingAverage *ma, int window_size) {
//     ma->index = 0;
//     ma->count = 0;
//     ma->sum = 0.0f;
//     for (int i = 0; i < window_size; i++) { // 使用傳入的 window_size
//         if (i < MAG_MOVING_AVG_SIZE) { // 避免越界
//              ma->values[i] = 0.0f;
//         }
//     }
// }

// /* 添加值到移动平均器并返回平均值 */
// static float moving_average_add(MovingAverage *ma, float value, int window_size) {
//     // 确保不超出数组大小 (MAG_MOVING_AVG_SIZE 是數組的實際大小)
//     if (window_size > MAG_MOVING_AVG_SIZE) window_size = MAG_MOVING_AVG_SIZE;

//     if (ma->count == window_size) {
//         ma->sum -= ma->values[ma->index];
//     } else {
//         ma->count++;
//     }

//     ma->values[ma->index] = value;
//     ma->sum += value;
//     ma->index = (ma->index + 1) % window_size;

//     return ma->sum / ma->count;
//     // return ma->sum >> ma->count;
// }


// // 使用新的驅動程式初始化 ICM20948
// static int new_icm20948_init(void) { // 重新命名以避免與舊的衝突 (如果舊的還在)
//     int ret;
//     printf("Starting NEW ICM20948 initialization\n");

//     if (!device_is_ready(dev_icm20948_spec.bus)) {
//         printf("I2C bus for ICM20948 (%s) is not ready!\n", dev_icm20948_spec.bus->name);
//         return -ENODEV;
//     }
//     printf("ICM20948 I2C Address: 0x%x\n", dev_icm20948_spec.addr);

//     // 使用新的驅動程式初始化
//     ret = icm20948_init(&my_icm_sensor, dev_icm20948_spec.bus, false, dev_icm20948_spec.addr);
//     if (ret != 0) {
//         printf("Failed to initialize ICM20948 core (new driver): %d\n", ret);
//         return ret;
//     }
//     printf("ICM20948 core (new driver) initialized successfully.\n");

//     // (可選) 檢查 WhoAmI
//     uint8_t whoami_val;
//     ret = icm20948_who_am_i(&my_icm_sensor, &whoami_val);
//     if (ret == 0) {
//         printf("ICM20948 WhoAmI (new driver): 0x%02X\n", whoami_val);
//     } else {
//         printf("Failed to read ICM20948 WhoAmI (new driver): %d\n", ret);
//     }

//     // 初始化磁力計 (AK09916) 使用新的驅動程式
//     ret = icm20948_init_magnetometer(&my_icm_sensor);
//     if (ret != 0) {
//         printf("Failed to initialize magnetometer (new driver): %d\n", ret);
//         return ret;
//     }

//     // 檢查磁力計 WhoAmI
//     uint16_t mag_id;
//     ret = icm20948_who_am_i_mag(&my_icm_sensor, &mag_id);
//     if (ret == 0) {
//         printf("AK09916 WhoAmI (new driver): 0x%04X (Expected: 0x4809 or 0x0948)\n", mag_id);
//         if (mag_id != 0x4809 && mag_id != 0x0948) {
//             printf("Warning: Magnetometer WhoAmI value mismatch!\n");
//         }
//     } else {
//         printf("Failed to read magnetometer WhoAmI (new driver): %d\n", ret);
//         return ret;
//     }
//     // 設置磁力計採樣頻率或模式 (如果驅動程式支援)
//     // 例如，假設驅動程式有函數可以設定磁力計模式
//     ret = icm20948_set_mag_op_mode(&my_icm_sensor, 0x08); // 0x08 代表更新率為100Hz
//     if (ret != 0) {
//         printf("Failed to set magnetometer mode (new driver): %d\n", ret);
//     }    

//     printf("Magnetometer (new driver) initialized successfully.\n");

//     // (可選) 根據需要設定感測器範圍或模式
//     // 例如，只啟用磁力計，禁用加速度計和陀螺儀 (如果驅動程式支援單獨控制)
//     // ret = icm20948_enable_acc(&my_icm_sensor, false);
//     // if(ret != 0) printf("Failed to disable accelerometer: %d\n", ret);
//     // ret = icm20948_enable_gyr(&my_icm_sensor, false);
//     // if(ret != 0) printf("Failed to disable gyroscope: %d\n", ret);
//     // 磁力計通常在 icm20948_init_magnetometer 中設定為連續模式

//     return 0;
// }

// // 初始化INA219 (保持不變)
// const struct device *ina219_init(void) {
//     const struct device *ina = DEVICE_DT_GET_ONE(ti_ina219);
//     const struct sensor_value fre = {1000, 0};
//     if (!device_is_ready(ina)) {
//         printf("Device %s is not ready.\n", ina->name);
//         return NULL;
//     }
//     printf("INA219 Device Ready\n");
//     return ina;
//     sensor_attr_set(ina, SENSOR_CHAN_CURRENT, SENSOR_ATTR_SAMPLING_FREQUENCY, &fre); // 設定取樣頻率
// }

// // 从INA219读取电流并更新共享数据 (保持不變)
// void ina219_read_thread_entry(void *arg1, void *arg2, void *arg3) { // 執行緒入口函式重命名
//     const struct device *ina = (const struct device *)arg1;
//     struct sensor_value current_sensor_val; // 避免與 shared_data 中的 current 混淆
//     // const struct sensor_value target_freq = {100, 0}; // 100 Hz, for example
//     int rc;
//     uint32_t start_time = k_uptime_get_32(); // 使用 32-bit uptime

//     MovingAverage current_filter; // 局部變數，或使用全域的 current_moving_avg
//     moving_average_init(&current_filter, CURRENT_FILTER_SIZE); // 使用 CURRENT_MOVING_AVG_SIZE
//     // sensor_attr_set(ina, SENSOR_CHAN_CURRENT, SENSOR_ATTR_SAMPLING_FREQUENCY, &target_freq); // 設定取樣頻率
//     while (1) {
//         uint32_t ina_thread_start = k_uptime_get_32();
//         k_mutex_lock(&i2c_mutex, K_FOREVER);
//         rc = sensor_sample_fetch(ina);
//         k_mutex_unlock(&i2c_mutex);
//         // uint32_t current_fetch_end = k_uptime_get_32();
//         // float fetch_duration = (float)(current_fetch_end - current_fetch_start)/1000;
//         // printf("INA219 Fetch Duration: %.4f ms\n", fetch_duration);
//         if (rc == 0) {
//             sensor_channel_get(ina, SENSOR_CHAN_CURRENT, &current_sensor_val);
//         }

//         if (rc != 0) {
//             printf("Could not fetch INA219 data: %d\n", rc);
//             k_sleep(SENSOR_SAMPLE_INTERVAL);
//             continue;
//         }

//         uint32_t current_time_ticks = k_uptime_get_32();
//         float current_mA = sensor_value_to_float(&current_sensor_val) * 1000.0f; // 轉換為 mA
//         // int current_mA = &current_sensor_val; // 轉換為 mA
//         float filtered_current = moving_average_add(&current_filter, current_mA, CURRENT_FILTER_SIZE);
//         // uint32_t current_average_time_ticks = k_uptime_get_32();
//         float elapsed_seconds = (float)(current_time_ticks - start_time) / MSEC_PER_SEC;
//         // float average_elapsed_seconds = (float)(current_average_time_ticks - current_time_ticks) / MSEC_PER_SEC;
//         // printf("Current Average Time: %.4f ms \n", average_elapsed_seconds);

//         k_mutex_lock(&motor_data_mutex, K_FOREVER);
//         motor_data.cur_sample_time = elapsed_seconds;
//         motor_data.current = filtered_current;
//         k_mutex_unlock(&motor_data_mutex);

//         k_sleep(SENSOR_SAMPLE_INTERVAL);
//         // k_msleep(10);
//         // k_usleep(1);
//         uint32_t ina_thread_end = k_uptime_get_32();
//         float ina_thread_duration = (float)(ina_thread_end - ina_thread_start)/1000;
//         printf("INA219 Thread Duration: %.4f ms\n", (double)ina_thread_duration);
//     }
// }

// // 从ICM20948读取磁场数据 (使用新的驅動程式)
// void icm20948_read_thread_entry(void *dummy0, void *dummy1, void *dummy2) { // 執行緒入口函式重命名
//     ARG_UNUSED(dummy0);
//     ARG_UNUSED(dummy1);
//     ARG_UNUSED(dummy2);

//     xyzFloat_t mag_xyz_data; // 用於儲存 XYZ 軸磁力計數據

//     moving_average_init(&mag_x_filter, MAG_MOVING_AVG_SIZE); // 初始化 X 軸濾波器

//     uint32_t start_time_icm = k_uptime_get_32();

//     while (1) {
//         uint32_t icm_thread_start = k_uptime_get_32();
//         int ret_icm;
//         float current_mag_x = 0.0f;
//         // int current_mag_x = 0;
        
//         // uint32_t icm_start = k_uptime_get_32();
//         k_mutex_lock(&i2c_mutex, K_FOREVER); // 加鎖保護 I2C 存取
//         // 1. 讀取感測器原始數據到驅動程式內部緩衝區
//         ret_icm = icm20948_read_sensor_data(&my_icm_sensor);
//         k_mutex_unlock(&i2c_mutex); // 解鎖
//         // uint32_t icm_end = k_uptime_get_32();
//         // float icm_duration = (float)(icm_end - icm_start)/1000;
//         // printf("ICM20948 Read Duration: %.4f ms\n", icm_duration);
//         if (ret_icm == 0) {
//             // 2. 從緩衝區獲取磁力計 XYZ 值
//             ret_icm = icm20948_get_mag_values(&my_icm_sensor, &mag_xyz_data);
//             if (ret_icm == 0) {
//                 current_mag_x = mag_xyz_data.y; // 獲取 Y 軸數據
//             } else {
//                 printf("Failed to get mag values from new ICM driver: %d\n", ret_icm);
//             }
//         } else {
//             printf("Failed to read sensor data from new ICM driver: %d\n", ret_icm);
//         }

//         uint32_t current_time_icm_ticks = k_uptime_get_32();
//         float elapsed_seconds_icm = (float)(current_time_icm_ticks - start_time_icm) / MSEC_PER_SEC;

//         // 對 X 軸磁場數據進行移動平均濾波
//         float filtered_mag_x_val = moving_average_add(&mag_x_filter, current_mag_x, MAG_MOVING_AVG_SIZE);
//         // uint32_t icm_average_end = k_uptime_get_32();
//         // float average_elapsed_seconds_icm = (float)(icm_average_end - current_time_icm_ticks) / MSEC_PER_SEC;
//         // printf("ICM20948 Mag X Average Time: %.4f ms \n", average_elapsed_seconds_icm);
//         // 更新共享數據結構中的磁場值 (X軸)
//         k_mutex_lock(&motor_data_mutex, K_FOREVER);
//         motor_data.mag_sample_time = elapsed_seconds_icm;
//         motor_data.filtered_mag_x = filtered_mag_x_val; // 儲存濾波後的 X 軸值
//         k_mutex_unlock(&motor_data_mutex);

//         // 使用 uart_mutex 保護 printf 輸出
//         k_mutex_lock(&uart_mutex, K_FOREVER);
//         printf("T1:%.4f\n"
//             "C:%.4f\n"
//             "T2:%.4f\n"
//             "M:%.4f\n", (double)motor_data.cur_sample_time, (double)motor_data.current, (double)elapsed_seconds_icm, (double)filtered_mag_x_val);
//         // printf("C:%.4f\n", motor_data.current);
//         // printf("T2:%.4f\n", elapsed_seconds_icm);
//         // printf("M:%.4f\n", filtered_mag_x_val);
//         k_mutex_unlock(&uart_mutex);
//         // fflush(stdout);

//         k_sleep(ICM_SENSOR_SAMPLE_INTERVAL);
//         // k_usleep(1);
//         // k_sleep(K_MSEC(10));  // 10ms延遲，100Hz採樣率，將icm20948的採樣頻率改為適合磁力計採樣頻率的間隔時間
//         uint32_t icm_thread_end = k_uptime_get_32();
//         float icm_thread_duration = (float)(icm_thread_end - icm_thread_start)/1000;
//         printf("ICM20948 Thread Duration: %.4f ms\n", (double)icm_thread_duration);
//     }
// }

// int main(void) {
//     printf("System Booting Up...\n");

//     // 初始化新的 ICM20948 驅動程式
//     if (new_icm20948_init() != 0) { // 呼叫新的初始化函式
//         printf("NEW ICM20948 initialization failed. Halting.\n");
//         return -1; // Or handle error appropriately
//     }

//     // 初始化INA219
//     const struct device *ina_dev = ina219_init();
//     if (ina_dev == NULL) {
//         printf("INA219 initialization failed. Halting.\n");
//         return -1;
//     }

//     // 初始化共享数据结构
//     k_mutex_lock(&motor_data_mutex, K_FOREVER);
//     motor_data.current = 0.0f;
//     motor_data.filtered_mag_x = 0.0f; // 初始化 X 軸
//     motor_data.mag_sample_time = 0.0f;
//     motor_data.cur_sample_time = 0.0f;
//     k_mutex_unlock(&motor_data_mutex);

//     // 创建 ICM20948 线程
//     k_tid_t icm_tid = k_thread_create(&icm20948_thread_data, icm20948_stack, ICM20948_STACK_SIZE,
//                         icm20948_read_thread_entry, NULL, NULL, NULL, // 使用新的執行緒入口函式
//                         ICM20948_PRIORITY, 0, K_NO_WAIT);
//     if (!icm_tid) {
//         printf("Failed to create ICM20948 thread.\n");
//     } else {
//         k_thread_name_set(icm_tid, "icm20948_reader");
//     }


//     // 创建 INA219 线程
//     k_tid_t ina_tid = k_thread_create(&ina219_thread_data, ina219_stack, INA219_STACK_SIZE,
//                         ina219_read_thread_entry, (void *)ina_dev, NULL, NULL, // 使用新的執行緒入口函式
//                         INA219_PRIORITY, 0, K_NO_WAIT);
//     if (!ina_tid) {
//         printf("Failed to create INA219 thread.\n");
//     } else {
//         k_thread_name_set(ina_tid, "ina219_reader");
//     }

//     printf("System initialization complete. Threads started.\n");
//     // fflush(stdout);

//     // 主執行緒可以進入睡眠或執行其他任務
//     while (1) {
//         k_usleep(1); // 例如，主執行緒每10秒喚醒一次做其他事情，或 K_FOREVER
//     }
//     return 0; // 不會執行到這裡
// }

/*使用自己寫的ina219驅動程式*/
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>

// 包含新的驅動程式標頭檔
#include "icm20948_zephyr.h"
#include "ina219.h"  // 新的 INA219 驅動程式

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* I2C 设备定义 */
#define I2C_NODE_ICM20948 DT_NODELABEL(icm20948)
static const struct i2c_dt_spec dev_icm20948_spec = I2C_DT_SPEC_GET(I2C_NODE_ICM20948);

/* 驅動程式實例 */
static icm20948_dev_t my_icm_sensor;
static struct ina219_data my_ina219;  // 新的 INA219 資料結構

/* 传感器数据刷新时间 */
#define SENSOR_SAMPLE_INTERVAL K_MSEC(1)
#define ICM_SENSOR_SAMPLE_INTERVAL K_MSEC(9)

/* INA219 配置參數 */
#define INA219_SHUNT_RESISTANCE    0.1f    // 0.1歐姆分流電阻
#define INA219_MAX_EXPECTED_CURRENT 0.5f   // 最大預期電流500mA
#define INA219_I2C_ADDRESS         0x40    // INA219 I2C 地址

/* 執行緒配置 */
#define ICM20948_STACK_SIZE 1024
#define INA219_STACK_SIZE 1024
#define ICM20948_PRIORITY 7
#define INA219_PRIORITY 7

K_THREAD_STACK_DEFINE(icm20948_stack, ICM20948_STACK_SIZE);
K_THREAD_STACK_DEFINE(ina219_stack, INA219_STACK_SIZE);
struct k_thread icm20948_thread_data;
struct k_thread ina219_thread_data;

K_MUTEX_DEFINE(uart_mutex);
K_MUTEX_DEFINE(i2c_mutex);

/* 移动平均窗口大小 */
#define CURRENT_FILTER_SIZE 5
#define SLOPE_FILTER_SIZE 5
#define MAG_MOVING_AVG_SIZE 5

/* 共享数据结构 */
struct shared_data {
    float current;
    float filtered_mag_x;
    float mag_sample_time;
    float cur_sample_time;
};

static struct shared_data motor_data;
static K_MUTEX_DEFINE(motor_data_mutex);

/*sliding window*/
/* 移动平均器结构体 */
typedef struct {
    float values[MAG_MOVING_AVG_SIZE];
    int index;
    int count;
    float sum;
} MovingAverage;

/* 数据滤波器 */ //針對移動平均濾波器所定義的結構體
// static MovingAverage mag_x_filter = {
//     .index = 0,
//     .count = 0,
//     .sum = 0.0f
// };

/* 初始化移动平均器 */
static void moving_average_init(MovingAverage *ma, int window_size) {
    ma->index = 0;
    ma->count = 0;
    ma->sum = 0.0f;
    for (int i = 0; i < window_size && i < MAG_MOVING_AVG_SIZE; i++) {
        ma->values[i] = 0.0f;
    }
}

/* 添加值到移动平均器并返回平均值 */
static float moving_average_add(MovingAverage *ma, float value, int window_size) {
    if (window_size > MAG_MOVING_AVG_SIZE) window_size = MAG_MOVING_AVG_SIZE;

    if (ma->count == window_size) {
        ma->sum -= ma->values[ma->index];
    } else {
        ma->count++;
    }

    ma->values[ma->index] = value;
    ma->sum += value;
    ma->index = (ma->index + 1) % window_size;

    return ma->sum / ma->count;
}

/* 一階低通濾波器結構體 */
typedef struct {
    float prev_output;     // Y(n-1) 上一次的輸出值
    float alpha;          // 濾波係數 α (0 < α <= 1)
    int initialized;      // 是否已初始化標誌
} FirstOrderFilter;

/* 數據濾波器 */
static FirstOrderFilter mag_x_filter = {
    .prev_output = 0.0f,
    .alpha = 0.05f,        // 預設α值，可根據需求調整
    .initialized = 0
};

static FirstOrderFilter mag_y_filter = {
    .prev_output = 0.0f,
    .alpha = 0.1f,
    .initialized = 0
};

static FirstOrderFilter mag_z_filter = {
    .prev_output = 0.0f,
    .alpha = 0.1f,
    .initialized = 0
};

/* 初始化一階濾波器 */
static void first_order_filter_init(FirstOrderFilter *filter, float alpha) {
    filter->prev_output = 0.0f;
    filter->alpha = alpha;
    filter->initialized = 0;
}

/* 設置濾波係數 */
static void first_order_filter_set_alpha(FirstOrderFilter *filter, float alpha) {
    // 確保 α 在有效範圍內
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    filter->alpha = alpha;
}

/* 應用一階低通濾波 */
static float first_order_filter_apply(FirstOrderFilter *filter, float input) {
    float output;
    
    // 如果是第一次運行，直接使用輸入值作為初始值
    if (!filter->initialized) {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }
    
    // 應用一階濾波公式: Y(n) = αX(n) + (1-α)Y(n-1)
    output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    
    // 更新上一次的輸出值
    filter->prev_output = output;
    
    return output;
}

/* 峰值檢測法 */
typedef enum {
    STATE_UNKNOWN,
    STATE_RISING,
    STATE_FALLING
} SignalState;

typedef struct {
    float threshold;      // 最小變化閾值
    SignalState state;
    float last_peak_time;
    float period;
    float last_mag;
    float current_sum;
    int current_count;
    int state_count;
} PeriodDetector;

bool detect_period(PeriodDetector *detector, float current_mag, float current_time, float current) {
    float diff = current_mag - detector->last_mag;
    
    // 使用閾值避免噪聲造成的誤判
    if (fabs(diff) < detector->threshold) {
        return false;
    }
    
    SignalState new_state = (diff > 0) ? STATE_RISING : STATE_FALLING;
    if (new_state == STATE_FALLING && detector->state == STATE_RISING && detector->state_count == 0) {
        detector->state_count = 1;
        detector->last_peak_time = current_time;
    }
    else if (new_state == STATE_FALLING && detector->state == STATE_RISING && detector->state_count == 1) {
        detector->state_count = 2;
    }
    if (detector->state_count == 1){
        detector->current_sum += current;
        detector->current_count++;
    }
    else if (detector->state_count == 2){
        detector->period = current_time - detector->last_peak_time;
        float average_current = detector->current_sum / detector->current_count;
        printf("----------Average Current during one period: %.4f mA----------\n", average_current);
        printf("----------Period: %.4f s----------\n", detector->period);
        printf("----------current time: %.4f s----------\n", current_time);
        printf("----------last peak time: %.4f s----------\n", detector->last_peak_time);
        detector->current_sum = 0.0f;
        detector->current_count = 0;
        detector->state_count = 0;
    }
    detector->last_mag = current_mag;
    detector->state = new_state;
    return false;
}

/* 週期檢測器實例 */
PeriodDetector cycle_detector = {
    .threshold = 0.5f,  // 根據磁場變化調整閾值
    .state = STATE_UNKNOWN,
    .last_peak_time = 0.0f,
    .period = 0.0f,
    .last_mag = 0.0f,
    .current_sum = 0.0f,
    .current_count = 0,
    .state_count = 0
};
/* 初始化 ICM20948 */
static int new_icm20948_init(void) {
    int ret;
    LOG_INF("Starting ICM20948 initialization");

    if (!device_is_ready(dev_icm20948_spec.bus)) {
        LOG_ERR("I2C bus for ICM20948 (%s) is not ready!", dev_icm20948_spec.bus->name);
        return -ENODEV;
    }
    LOG_INF("ICM20948 I2C Address: 0x%x", dev_icm20948_spec.addr);

    ret = icm20948_init(&my_icm_sensor, dev_icm20948_spec.bus, false, dev_icm20948_spec.addr);
    if (ret != 0) {
        LOG_ERR("Failed to initialize ICM20948 core: %d", ret);
        return ret;
    }
    LOG_INF("ICM20948 core initialized successfully");

    uint8_t whoami_val;
    ret = icm20948_who_am_i(&my_icm_sensor, &whoami_val);
    if (ret == 0) {
        LOG_INF("ICM20948 WhoAmI: 0x%02X", whoami_val);
    }

    ret = icm20948_init_magnetometer(&my_icm_sensor);
    if (ret != 0) {
        LOG_ERR("Failed to initialize magnetometer: %d", ret);
        return ret;
    }

    uint16_t mag_id;
    ret = icm20948_who_am_i_mag(&my_icm_sensor, &mag_id);
    if (ret == 0) {
        LOG_INF("AK09916 WhoAmI: 0x%04X (Expected: 0x4809 or 0x0948)", mag_id);
    }

    ret = icm20948_set_mag_op_mode(&my_icm_sensor, 0x08);
    if (ret != 0) {
        LOG_WRN("Failed to set magnetometer mode: %d", ret);
    }

    LOG_INF("Magnetometer initialized successfully");
    return 0;
}

/* 初始化 INA219 - 使用新的驅動程式 */
static int new_ina219_init(void) {
    int ret;
    LOG_INF("Starting INA219 initialization");

    // 獲取 I2C 裝置
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device for INA219 not ready");
        return -ENODEV;
    }

    // 初始化 INA219
    ret = ina219_init(&my_ina219, i2c_dev, INA219_I2C_ADDRESS);
    if (ret != 0) {
        LOG_ERR("Failed to initialize INA219: %d", ret);
        return ret;
    }

    // 設定校準值
    ret = ina219_set_calibration(&my_ina219, INA219_SHUNT_RESISTANCE, INA219_MAX_EXPECTED_CURRENT);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 calibration: %d", ret);
        return ret;
    }

    // 設定 ADC 模式為 128 樣本平均（最高精度）
    // 這對於您的 AC/DC 分析很重要
    ret = ina219_set_adc_mode(&my_ina219, INA219_ADC_12BIT_16S, INA219_ADC_12BIT_16S);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 ADC mode: %d", ret);
        return ret;
    }
    LOG_INF("INA219 ADC mode set to 128-sample averaging");

    // 設定匯流排電壓範圍為 32V
    ret = ina219_set_bus_range(&my_ina219, INA219_BUSRANGE_32V);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 bus range: %d", ret);
        return ret;
    }

    // 設定 PGA 增益為 320mV
    ret = ina219_set_gain(&my_ina219, INA219_GAIN_40MV);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 gain: %d", ret);
        return ret;
    }

    // 設定連續測量模式
    ret = ina219_set_mode(&my_ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
    if (ret != 0) {
        LOG_ERR("Failed to set INA219 mode: %d", ret);
        return ret;
    }

    LOG_INF("INA219 initialized successfully");
    return 0;
}

/* INA219 讀取執行緒 - 使用新的驅動程式 */
void ina219_read_thread_entry(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    uint32_t start_time = k_uptime_get_32();
    MovingAverage current_filter;
    moving_average_init(&current_filter, CURRENT_FILTER_SIZE);

    LOG_INF("INA219 thread started");

    while (1) {
        uint32_t ina_thread_start = k_uptime_get_32();

        // 讀取電流值（已包含 I2C 通訊）
        k_mutex_lock(&i2c_mutex, K_FOREVER);
        float current_mA = ina219_read_current(&my_ina219);
        float bus_voltage = ina219_read_bus_voltage(&my_ina219);
        float shunt_voltage = ina219_read_shunt_voltage(&my_ina219);
        float power_mW = ina219_read_power(&my_ina219);
        k_mutex_unlock(&i2c_mutex);

        // 應用移動平均濾波
        float filtered_current = moving_average_add(&current_filter, current_mA, CURRENT_FILTER_SIZE);

        uint32_t current_time_ticks = k_uptime_get_32();
        float elapsed_seconds = (float)(current_time_ticks - start_time) / 1000.0f;

        // 更新共享資料
        k_mutex_lock(&motor_data_mutex, K_FOREVER);
        motor_data.cur_sample_time = elapsed_seconds;
        motor_data.current = filtered_current;
        k_mutex_unlock(&motor_data_mutex);

        // 偵錯輸出（可選）
        if ((current_time_ticks % 1000) < 50) { // 每秒輸出一次
            LOG_DBG("INA219: Current=%.2f mA, Voltage=%.2f V, Shunt=%.2f mV, Power=%.2f mW",
                    (double)current_mA, (double)bus_voltage, 
                    (double)shunt_voltage, (double)power_mW);
        }

        k_sleep(SENSOR_SAMPLE_INTERVAL);

        uint32_t ina_thread_end = k_uptime_get_32();
        float ina_thread_duration = (float)(ina_thread_end - ina_thread_start);
        
        // 偵錯輸出執行時間（可選）
        if (ina_thread_duration > 10.0f) { // 只在超過 10ms 時輸出
            LOG_DBG("INA219 Thread Duration: %.2f ms", (double)ina_thread_duration);
        }
    }
}

/* ICM20948 讀取執行緒（保持原樣） */
void icm20948_read_thread_entry(void *dummy0, void *dummy1, void *dummy2) {
    ARG_UNUSED(dummy0);
    ARG_UNUSED(dummy1);
    ARG_UNUSED(dummy2);

    xyzFloat_t mag_xyz_data;
    // moving_average_init(&mag_x_filter, MAG_MOVING_AVG_SIZE);
    first_order_filter_init(&mag_x_filter, 0.1f); // 初始化一階濾波器，α值可調整
    uint32_t start_time_icm = k_uptime_get_32();

    LOG_INF("ICM20948 thread started");

    while (1) {
        uint32_t icm_thread_start = k_uptime_get_32();
        int ret_icm;
        float current_mag_x = 0.0f;
        
        k_mutex_lock(&i2c_mutex, K_FOREVER);
        ret_icm = icm20948_read_sensor_data(&my_icm_sensor);
        k_mutex_unlock(&i2c_mutex);

        if (ret_icm == 0) {
            ret_icm = icm20948_get_mag_values(&my_icm_sensor, &mag_xyz_data);
            if (ret_icm == 0) {
                current_mag_x = mag_xyz_data.y;
            } else {
                LOG_ERR("Failed to get mag values: %d", ret_icm);
            }
        } else {
            LOG_ERR("Failed to read sensor data: %d", ret_icm);
        }

        uint32_t current_time_icm_ticks = k_uptime_get_32();
        float elapsed_seconds_icm = (float)(current_time_icm_ticks - start_time_icm) / 1000.0f;

        // float filtered_mag_x_val = moving_average_add(&mag_x_filter, current_mag_x, MAG_MOVING_AVG_SIZE);
        float filtered_mag_x_val = first_order_filter_apply(&mag_x_filter, current_mag_x);

        k_mutex_lock(&motor_data_mutex, K_FOREVER);
        motor_data.mag_sample_time = elapsed_seconds_icm;
        motor_data.filtered_mag_x = filtered_mag_x_val;
        k_mutex_unlock(&motor_data_mutex);
        // 輸出資料
        k_mutex_lock(&uart_mutex, K_FOREVER);
        printf("T1:%.4f\n"
               "C:%.4f\n"
               "T2:%.4f\n"
               "M:%.4f\n", 
               (double)motor_data.cur_sample_time, 
               (double)motor_data.current, 
               (double)elapsed_seconds_icm, 
               (double)filtered_mag_x_val);
        k_mutex_unlock(&uart_mutex);
        bool detected_period = detect_period(&cycle_detector, filtered_mag_x_val, elapsed_seconds_icm, motor_data.current);

        k_sleep(ICM_SENSOR_SAMPLE_INTERVAL);

        uint32_t icm_thread_end = k_uptime_get_32();
        float icm_thread_duration = (float)(icm_thread_end - icm_thread_start);
        
        if (icm_thread_duration > 15.0f) {
            LOG_DBG("ICM20948 Thread Duration: %.2f ms", (double)icm_thread_duration);
        }
    }
}

/* 主函數 */
int main(void) {
    LOG_INF("System Booting Up...");

    // 初始化 ICM20948
    if (new_icm20948_init() != 0) {
        LOG_ERR("ICM20948 initialization failed. Halting.");
        return -1;
    }

    // 初始化 INA219 - 使用新的驅動程式
    if (new_ina219_init() != 0) {
        LOG_ERR("INA219 initialization failed. Halting.");
        return -1;
    }

    // 初始化共享資料結構
    k_mutex_lock(&motor_data_mutex, K_FOREVER);
    motor_data.current = 0.0f;
    motor_data.filtered_mag_x = 0.0f;
    motor_data.mag_sample_time = 0.0f;
    motor_data.cur_sample_time = 0.0f;
    k_mutex_unlock(&motor_data_mutex);

    // 創建 ICM20948 執行緒
    k_tid_t icm_tid = k_thread_create(&icm20948_thread_data, 
                                      icm20948_stack, 
                                      ICM20948_STACK_SIZE,
                                      icm20948_read_thread_entry, 
                                      NULL, NULL, NULL,
                                      ICM20948_PRIORITY, 
                                      0, 
                                      K_NO_WAIT);
    if (!icm_tid) {
        LOG_ERR("Failed to create ICM20948 thread");
    } else {
        k_thread_name_set(icm_tid, "icm20948_reader");
    }

    // 創建 INA219 執行緒
    k_tid_t ina_tid = k_thread_create(&ina219_thread_data, 
                                      ina219_stack, 
                                      INA219_STACK_SIZE,
                                      ina219_read_thread_entry, 
                                      NULL, NULL, NULL,
                                      INA219_PRIORITY, 
                                      0, 
                                      K_NO_WAIT);
    if (!ina_tid) {
        LOG_ERR("Failed to create INA219 thread");
    } else {
        k_thread_name_set(ina_tid, "ina219_reader");
    }

    LOG_INF("System initialization complete. Threads started.");

    // 主執行緒進入循環，可以用於動態調整 ADC 模式
    // uint32_t loop_count = 0;
    // while (1) {
    //     k_sleep(K_SECONDS(10)); // 每 10 秒執行一次
    //     loop_count++;

    //     // 示例：根據需求動態調整 ADC 模式
    //     if (loop_count % 6 == 0) { // 每分鐘調整一次
    //         LOG_INF("Adjusting INA219 ADC mode for testing...");
            
    //         k_mutex_lock(&i2c_mutex, K_FOREVER);
            
    //         // 測試不同的 ADC 模式
    //         if (loop_count % 12 == 0) {
    //             // 切換到快速模式（較低精度但更快的響應）
    //             ina219_set_adc_mode(&my_ina219, INA219_ADC_12BIT_2S, INA219_ADC_12BIT_2S);
    //             LOG_INF("Switched to fast ADC mode (2 samples)");
    //         } else {
    //             // 切換回高精度模式
    //             ina219_set_adc_mode(&my_ina219, INA219_ADC_12BIT_128S, INA219_ADC_12BIT_128S);
    //             LOG_INF("Switched to high precision ADC mode (128 samples)");
    //         }
            
    //         k_mutex_unlock(&i2c_mutex);
    //     }

    //     // 輸出系統狀態
    //     LOG_INF("System running... Loop count: %d", loop_count);
    // }

    return 0;
}