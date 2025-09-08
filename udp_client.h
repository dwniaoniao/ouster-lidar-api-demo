#ifndef UDP_CLIENT_H

#define UDP_CLIENT_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

#define LIDAR_PACKET_BUFFER_SIZE 65535
#define IMU_PACKET_BUFFER_SIZE 48

typedef enum os_status_t{
    OS_FAIL = 0,
    OS_SUCCESS
} os_status_t;

typedef struct imu_packet_t{
    uint64_t sys_ts;
    uint64_t accel_ts;
    uint64_t gyro_ts;
    float la_x;
    float la_y;
    float la_z;
    float av_x;
    float av_y;
    float av_z;
} imu_packet_t;

typedef struct __attribute__((packed)) lidar_packet_header_t{
    uint16_t packet_type;
    uint16_t frame_id;
    uint64_t init_id : 24;
    uint64_t serial_no : 40;
    uint32_t alert_flags : 8;
    uint32_t : 24;
    uint32_t shutdown_countdown : 8;
    uint32_t shot_limiting_countdown : 8;
    uint32_t shutdown_status : 4;
    uint32_t : 4;
    uint32_t shot_limiting : 4;
    uint32_t : 4;
    uint32_t : 32;
    uint32_t : 32;
    uint32_t : 32;
} lidar_packet_header_t; 

typedef struct __attribute__((packed)) lidar_packet_footer_t{
    uint64_t : 64;
    uint64_t : 64;
    uint64_t : 64;
    uint64_t e2e_crc;
} lidar_packet_footer_t;

typedef struct __attribute__((packed)) column_header_t{
    uint64_t timestamp;
    uint32_t measurement_id : 16;
    uint32_t status : 1;
    uint32_t : 15;
} column_header_t;

typedef struct __attribute__((packed)) channel_data_sr{
    uint32_t range : 19;
    uint32_t : 13;
    uint32_t reflectivity : 8;
    uint32_t : 8;
    uint32_t signal : 16;
    uint32_t nir : 16;
    uint32_t : 16;
} channel_data_sr;

typedef struct __attribute__((packed)) channel_data_ld{
    uint32_t range : 15;
    uint32_t : 1;
    uint32_t reflectivity : 8;
    uint32_t nir : 8;
} channel_data_ld;

typedef struct __attribute__((packed)) channel_data_dr{
    uint32_t range_ret1 : 19;
    uint32_t : 5;
    uint32_t reflectivity_ret1 : 8;
    uint32_t range_ret2 : 19;
    uint32_t : 5;
    uint32_t reflectivity_ret2 : 8;
    uint32_t signal_ret1 : 16;
    uint32_t signal_ret2 : 16;
    uint16_t nir;
    uint16_t : 16;
} channel_data_dr;

typedef struct __attribute__((packed)) column_data_t{
    column_header_t column_header_block;
    void *channel_data_block;
} column_data_t;

typedef struct lidar_packet_t{
    lidar_packet_header_t packet_header;
    column_data_t *column_data;
    lidar_packet_footer_t packet_footer;
} lidar_packet_t;

typedef enum udp_profile_lidar_t{
    RNG19_RFL8_SIG16_NIR16,
    RNG15_RFL8_NIR8,
    RNG19_RFL8_SIG16_NIR16_DUAL
} udp_profile_lidar_t;

typedef struct sensor_info_t{
    size_t columns_per_packet;
    size_t pixels_per_column;
    size_t columns_per_frame;
    udp_profile_lidar_t udp_profile_lidar;
    double beam_to_lidar_transform[4][4];
    double *beam_altitude_angles;
    double *beam_azimuth_angles;
} sensor_info_t;

typedef struct lidar_scan_t{
    size_t w;
    size_t h;
    uint16_t frame_id;
    uint64_t *timestamp;
    uint16_t *measurement_id;
    uint32_t *status;
    uint32_t *rng;
    uint8_t  *ref;
    uint16_t *sig;
    uint16_t *nir;
    uint32_t *rng2;
    uint8_t  *ref2;
    uint16_t *sig2;
} lidar_scan_t;

typedef struct xyz_t{
    double x;
    double y;
    double z;
} xyz_t;

int os_init_udp_client(uint16_t port);
void os_close_udp_client(int sockfd);
ssize_t os_udp_client_recv(int sockfd, void *buf, size_t len);
os_status_t os_udp_client_get_imu_packet(int sockfd, imu_packet_t *imu_packet);
os_status_t os_udp_client_get_lidar_packet(int sockfd, lidar_packet_t *lidar_packet);
lidar_packet_t* os_alloc_lidar_packet();
void os_delete_lidar_packet(lidar_packet_t *packet);
lidar_scan_t* os_alloc_lidar_scan();
os_status_t os_udp_client_get_lidar_scan(int sockfd, lidar_scan_t *scan);
void os_delete_lidar_scan(lidar_scan_t *scan);
void os_batch_packet_to_scan(lidar_packet_t *packet, lidar_scan_t *scan);
void os_destagger(void *buf, void *buf_destaggered,
    int *pixel_shift_by_row, size_t buf_len, size_t width, size_t height);
xyz_t* os_cartesian(lidar_scan_t *scan, size_t *count);
void os_set_sensor_info(sensor_info_t *info);

#endif

