#include <stdio.h>
#include <stdlib.h>
#include "udp_client.h"
#include "curl_client.h"
#include "cvwrapper.h"
#include "exporter.h"

static double beam_altitude_angles[] = {
    0.12, -0.23, -0.6, -0.96, -1.29, -1.64, -2.0, -2.35,
    -2.69, -3.05, -3.4, -3.74, -4.09, -4.45, -4.8, -5.16,
    -5.49, -5.84, -6.19, -6.55, -6.88, -7.23, -7.57, -7.93,
    -8.28, -8.61, -8.96, -9.29, -9.64, -9.98, -10.32, -10.66,
    -11.0, -11.33, -11.68, -12.02, -12.35, -12.68, -13.03, -13.36,
    -13.7, -14.02, -14.34, -14.68, -15.02, -15.33, -15.65, -15.96,
    -16.31, -16.63, -16.95, -17.26, -17.6, -17.9, -18.23, -18.53,
    -18.87, -19.17, -19.47, -19.76, -20.12, -20.41, -20.69, -20.99
};

static double beam_azimuth_angles[] = {
    4.21, 1.39, -1.43, -4.22, 4.2, 1.39, -1.42, -4.2,
    4.21, 1.39, -1.42, -4.2, 4.21, 1.4, -1.41, -4.21,
    4.21, 1.4, -1.41, -4.21, 4.21, 1.4, -1.39, -4.2,
    4.2, 1.4, -1.4, -4.18, 4.22, 1.41, -1.4, -4.18,
    4.22, 1.43, -1.39, -4.19, 4.23, 1.42, -1.39, -4.19,
    4.23, 1.43, -1.38, -4.19, 4.23, 1.43, -1.37, -4.16,
    4.24, 1.44, -1.38, -4.16, 4.25, 1.44, -1.37, -4.16,
    4.24, 1.45, -1.37, -4.15, 4.25, 1.45, -1.36, -4.15
};

static int pixel_shift_by_row[] = {
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12,
    12, 4, -4, -12, 12, 4, -4, -12
};

static sensor_info_t info = {
    16,   // columns_per_packet
    64,   // pixels_per_column
    1024, // columns_per_frame
    RNG19_RFL8_SIG16_NIR16, // udp_profile_lidar

    // beam_to_lidar_transform
    {1.0, 0.0, 0.0, 15.806,
     0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0},

    beam_altitude_angles,
    beam_azimuth_angles
};

int get_scan_test()
{
    uint16_t port = 7502;

    int sockfd = os_init_udp_client(port);
    if(sockfd == -1){
        perror("os_init_udp_client\n");
        return EXIT_FAILURE;
    }
    os_set_sensor_info(&info);
    lidar_scan_t *scan = os_alloc_lidar_scan();
    if(!scan){
        perror("os_alloc_lidar_scan\n");
        return EXIT_FAILURE;
    }
    if(os_udp_client_get_lidar_scan(sockfd, scan) == OS_FAIL){
        perror("os_udp_client_get_lidar_scan\n");
        os_delete_lidar_scan(scan);
        return EXIT_FAILURE;
    }
    uint8_t ref_destaggered[1024 * 64];
    uint16_t sig_destaggered[1024 * 64];
    uint16_t nir_destaggered[1024 * 64];
    uint32_t rng_destaggered[1024 * 64];
    os_destagger((void *)scan->ref, (void *)ref_destaggered, pixel_shift_by_row, 1024 * 64, 1024, 64);
    os_destagger((void *)scan->sig, (void *)sig_destaggered, pixel_shift_by_row, 1024 * 64 * 2, 1024, 64);
    os_destagger((void *)scan->nir, (void *)nir_destaggered, pixel_shift_by_row, 1024 * 64 * 2, 1024, 64);
    os_destagger((void *)scan->rng, (void *)rng_destaggered, pixel_shift_by_row, 1024 * 64 * 4, 1024, 64);
    create_window();
    display_image_8u(ref_destaggered, 1024, 64);
    wait_key(0);
    display_image_16u(sig_destaggered, 1024, 64);
    wait_key(0);
    display_image_16u(nir_destaggered, 1024, 64);
    wait_key(0);
    display_image_32u(rng_destaggered, 1024, 64);
    wait_key(0);
    destroy_all_windows();

    size_t count;
    xyz_t *xyz = os_cartesian(scan, &count);
    if(xyz){
        write_point_cloud(xyz, count);
        free(xyz);
    }
    os_delete_lidar_scan(scan);

    return EXIT_SUCCESS;
}

static int get_user_data(CURL *curl)
{
    int result;
    printf("Getting user data...\n");
    char *url = "http://ouster/api/v1/user/data";
    struct memory mem = {0};
    CURLcode res = os_curl_get(curl, url, &mem);
    if(res == CURLE_OK){
        printf("%s\n", mem.response);
        result = EXIT_SUCCESS;
    } else{
        perror("Error: get user data failed.");
        result = EXIT_FAILURE;
    }
    free(mem.response);
    return result;
}

static int set_user_data(CURL *curl, char *str)
{
    printf("Setting user data...\n");
    char *url = "http://ouster/api/v1/user/data";
    CURLcode res = os_curl_put(curl, url, str);
    if(res == CURLE_OK){
        printf("Set user data success.\n");
        return EXIT_SUCCESS;
    } else{
        perror("Error: set user data failed.");
        return EXIT_FAILURE;
    }
}

static int delete_user_data(CURL *curl)
{
    printf("Deleting user data...\n");
    char *url = "http://ouster/api/v1/user/data";
    CURLcode res = os_curl_delete(curl, url);
    if(res == CURLE_OK){
        printf("Delete user data success.\n");
        return EXIT_SUCCESS;
    } else{
        perror("Error: delete user data failed.");
        return EXIT_FAILURE;
    }
}

static int get_sensor_config(CURL *curl)
{
    int result;
    printf("Getting sensor config...\n");
    char *url = "http://ouster/api/v1/sensor/config";
    struct memory mem = {0};
    CURLcode res = os_curl_get(curl, url, &mem);
    if(res == CURLE_OK){
        printf("%s\n", mem.response);
        result = EXIT_SUCCESS;
    } else{
        perror("Error: get sensor config failed.");
        result = EXIT_FAILURE;
    }
    free(mem.response);
    return result;
}

static int set_sensor_config(CURL *curl, char *str)
{
    printf("Setting sensor config...\n");
    char *url = "http://ouster/api/v1/sensor/config";
    CURLcode res = os_curl_post(curl, url, str);
    if(res == CURLE_OK){
        printf("Set sensor config success.\n");
        return EXIT_SUCCESS;
    } else{
        perror("Error: set sensor config failed.");
        return EXIT_FAILURE;
    }
}

static int curl_client_test()
{
    CURL *curl = os_init_curl_client();
    if(!curl){
        perror("Error: initiate curl client failed.");
        return EXIT_FAILURE;
    }
    do{
        if(get_sensor_config(curl) == EXIT_FAILURE) break;
        char *config_str = "{\"lidar_mode\" : \"512x10\"}";
        if(set_sensor_config(curl, config_str) == EXIT_FAILURE) break;
        if(get_sensor_config(curl) == EXIT_FAILURE) break;
        if(get_user_data(curl) == EXIT_FAILURE) break;
        char *user_data_str = "\"my own data\"";
        if(set_user_data(curl, user_data_str) == EXIT_FAILURE) break;
        if(get_user_data(curl) == EXIT_FAILURE) break;
        if(delete_user_data(curl) == EXIT_FAILURE) break;
        if(get_user_data(curl) == EXIT_FAILURE) break;
        os_deinit_curl_client(curl);
        return EXIT_SUCCESS;
    } while(0);

    os_deinit_curl_client(curl);
    return EXIT_FAILURE;
}

static int read_imu_data()
{
    uint16_t port = 7503;
    imu_packet_t p = {0};
    int count = 100;

    int sockfd = os_init_udp_client(port);
    if(sockfd == -1){
        perror("os_init_udp_client\n");
        return EXIT_FAILURE;
    }
    while(count--){
        if(os_udp_client_get_imu_packet(sockfd, &p) != OS_SUCCESS){
            os_close_udp_client(sockfd);
            return EXIT_FAILURE;
        } else{
            printf("%f %f %f %f %f %f\n", p.la_x,
                                          p.la_y,
                                          p.la_z,
                                          p.av_x,
                                          p.av_y,
                                          p.av_z);
        }
    }
    os_close_udp_client(sockfd);
    return EXIT_SUCCESS;
}

int main()
{
    // return curl_client_test();
    // return read_imu_data();
    return get_scan_test();
}

