#include "udp_client.h"
#include "crc.h"

#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include <inttypes.h>

static sensor_info_t sensor_info = {0};

void os_set_sensor_info(sensor_info_t *info)
{
    sensor_info.columns_per_packet = info->columns_per_packet;
    sensor_info.pixels_per_column = info->pixels_per_column;
    sensor_info.columns_per_frame = info->columns_per_frame;
    sensor_info.udp_profile_lidar = info->udp_profile_lidar;
    memcpy((void *)sensor_info.beam_to_lidar_transform,
           (void *)info->beam_to_lidar_transform, sizeof(double) * 16);
    sensor_info.beam_altitude_angles = info->beam_altitude_angles;
    sensor_info.beam_azimuth_angles = info->beam_azimuth_angles;
}

void os_destagger(void *buf, void *buf_destaggered,
    int *pixel_shift_by_row, size_t buf_len, size_t width, size_t height)
{
    void *p = buf;
    void *p2 = buf_destaggered;
    ssize_t shift;
    size_t element_size = buf_len / (width * height);
    size_t byte_per_column = width * element_size;
    for(size_t i = 0; i < height; i++){
        shift = pixel_shift_by_row[i] * element_size;
        if(shift <= 0){
            shift = -shift;
            memcpy(p2, p + shift, byte_per_column - shift);
            memcpy(p2 + byte_per_column - shift, p, shift);
        } else{
            memcpy(p2, p + byte_per_column - shift, shift);
            memcpy(p2 + shift, p, byte_per_column - shift);
        }
        p += width * element_size;
        p2 += width * element_size;
    }
}

int os_init_udp_client(uint16_t port)
{
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%u", port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    struct addrinfo *servinfo;
    int rv = getaddrinfo(NULL, port_str, &hints, &servinfo);
    if(rv){
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return -1;
    }

    int sockfd;
    struct addrinfo *p;
    for(p = servinfo; p != NULL; p = p->ai_next){
        if((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1){
            continue;
        }
        if(bind(sockfd, p->ai_addr, p->ai_addrlen) == -1){
            continue;
        }
        break;
    }

    if(p == NULL){
        fprintf(stderr, "fail to create and bind socket.\n");
        return -1;
    }

    freeaddrinfo(servinfo);
    return sockfd;
}

void os_close_udp_client(int sockfd)
{
    close(sockfd);
}

ssize_t os_udp_client_recv(int sockfd, void *buf, size_t len)
{
    return recvfrom(sockfd, buf, len, 0, NULL, NULL);
}

os_status_t os_udp_client_get_imu_packet(int sockfd, imu_packet_t *imu_packet)
{
    uint8_t buf[IMU_PACKET_BUFFER_SIZE];
    if(os_udp_client_recv(sockfd, (void *)buf, sizeof(buf))){
        memcpy((void *)imu_packet, (void *)buf, 8);
        memcpy((void *)imu_packet + 8, (void *)buf + 8, 8);
        memcpy((void *)imu_packet + 16, (void *)buf + 16, 8);
        memcpy((void *)imu_packet + 24, (void *)buf + 24, 4);
        memcpy((void *)imu_packet + 28, (void *)buf + 28, 4);
        memcpy((void *)imu_packet + 32, (void *)buf + 32, 4);
        memcpy((void *)imu_packet + 36, (void *)buf + 36, 4);
        memcpy((void *)imu_packet + 40, (void *)buf + 40, 4);
        memcpy((void *)imu_packet + 44, (void *)buf + 44, 4);
        return OS_SUCCESS;
    }
    return OS_FAIL;
}

os_status_t os_udp_client_get_lidar_packet(int sockfd, lidar_packet_t *lidar_packet)
{
    uint8_t buf[LIDAR_PACKET_BUFFER_SIZE];
    size_t len_per_pixel;
    if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16){
        len_per_pixel = sizeof(channel_data_sr);
    } else if(sensor_info.udp_profile_lidar == RNG15_RFL8_NIR8){
        len_per_pixel = sizeof(channel_data_ld);
    } else if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16_DUAL){
        len_per_pixel = sizeof(channel_data_dr);
    }

    size_t header_len = sizeof(lidar_packet_header_t);
    size_t footer_len = sizeof(lidar_packet_footer_t);
    size_t column_header_len = sizeof(column_header_t);
    size_t column_data_len = column_header_len + len_per_pixel *
        sensor_info.pixels_per_column;
    size_t total_column_data_len = column_data_len * sensor_info.columns_per_packet;

    size_t header_offset = 0;
    size_t column_data_offset = header_len;
    size_t footer_offset = column_data_offset + total_column_data_len;

    size_t packet_len = header_len + footer_len + total_column_data_len;
    ssize_t len = os_udp_client_recv(sockfd, (void *)buf, sizeof(buf));
    if(len){
        // printf("%d %d\n", len, packet_len);
        assert(len == packet_len);
        memcpy((void *)&lidar_packet->packet_header, (void *)buf + header_offset, header_len);
        memcpy((void *)&lidar_packet->packet_footer, (void *)buf + footer_offset, footer_len);

        uint64_t crc = calculate_crc((uint8_t *)buf, len - 8);
        // printf("0x%" PRIx64 " ", crc);
        // printf("0x%" PRIx64 "\n", lidar_packet->packet_footer.e2e_crc);
        if(crc != lidar_packet->packet_footer.e2e_crc)
            return OS_FAIL;

        column_data_t *p = lidar_packet->column_data;
        for(size_t i = 0; i < sensor_info.columns_per_packet; i++){
            memcpy((void *)&(p[i].column_header_block),
                    (void *)buf + column_data_offset + column_data_len * i,
                    column_header_len);
            memcpy(p[i].channel_data_block,
                    (void *)buf + column_data_offset + column_data_len * i + column_header_len,
                    column_data_len - column_header_len);
        }
        return OS_SUCCESS;
    }
    return OS_FAIL;
}

lidar_packet_t* os_alloc_lidar_packet()
{
    lidar_packet_t* packet = (lidar_packet_t *)malloc(sizeof(lidar_packet_t));
    if(!packet) return NULL;

    packet->column_data = (column_data_t *)malloc(sizeof(column_data_t) *
            sensor_info.columns_per_packet);
    if(!packet->column_data){
        free(packet);
        return NULL;
    }

    size_t len_per_pixel;
    if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16){
        len_per_pixel = sizeof(channel_data_sr);
    } else if(sensor_info.udp_profile_lidar == RNG15_RFL8_NIR8){
        len_per_pixel = sizeof(channel_data_ld);
    } else if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16_DUAL){
        len_per_pixel = sizeof(channel_data_dr);
    }

    column_data_t *p = packet->column_data;
    for(size_t i = 0; i < sensor_info.columns_per_packet; i++){
        p[i].channel_data_block = malloc(sensor_info.pixels_per_column *
                len_per_pixel);
        if(!p[i].channel_data_block){
            while(i){
                i--;
                free(p[i].channel_data_block);
            }
            free(packet->column_data);
            free(packet);
            return NULL;
        }
    }

    return packet;
}

void os_delete_lidar_packet(lidar_packet_t *packet)
{
    for(size_t i = 0; i < sensor_info.columns_per_packet; i++){
        free(packet->column_data[i].channel_data_block);
    }
    free(packet->column_data);
    free(packet);
}

void os_batch_packet_to_scan(lidar_packet_t *packet, lidar_scan_t *scan)
{
    size_t offset_s;
    void *p;
    channel_data_sr data_sr;
    channel_data_ld data_ld;
    channel_data_dr data_dr;
    for(size_t i = 0; i < sensor_info.columns_per_packet; i++){
        uint32_t m_id = packet->column_data[i].column_header_block.measurement_id;
        uint64_t ts = packet->column_data[i].column_header_block.timestamp;
        uint32_t status = packet->column_data[i].column_header_block.status;
        if(!status) continue;
        scan->status[m_id] = status;
        scan->measurement_id[m_id] = m_id;
        scan->timestamp[m_id] = ts;
        p = packet->column_data[i].channel_data_block;
        for(size_t j = 0; j < sensor_info.pixels_per_column; j++){
            offset_s = scan->w * j + m_id;
            if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16){
                data_sr = ((channel_data_sr *)p)[j];
                *(scan->rng + offset_s) = data_sr.range;
                *(scan->ref + offset_s) = data_sr.reflectivity;
                *(scan->sig + offset_s) = data_sr.signal;
                *(scan->nir + offset_s) = data_sr.nir;
            } else if(sensor_info.udp_profile_lidar == RNG15_RFL8_NIR8){
                data_ld = ((channel_data_ld *)p)[j];
                *(scan->rng + offset_s) = data_ld.range;
                *(scan->ref + offset_s) = data_ld.reflectivity;
                *(scan->nir + offset_s) = data_ld.nir;
            } else if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16_DUAL){
                data_dr = ((channel_data_dr *)p)[j];
                *(scan->rng + offset_s) = data_dr.range_ret1;
                *(scan->ref + offset_s) = data_dr.reflectivity_ret1;
                *(scan->sig + offset_s) = data_dr.signal_ret1;
                *(scan->nir + offset_s) = data_dr.nir;
                *(scan->rng2 + offset_s) = data_dr.range_ret2;
                *(scan->ref2 + offset_s) = data_dr.reflectivity_ret2;
                *(scan->sig2 + offset_s) = data_dr.signal_ret2;
            }
        }
    }
}

os_status_t os_udp_client_get_lidar_scan(int sockfd, lidar_scan_t *scan)
{
    lidar_packet_t *packet = os_alloc_lidar_packet();
    if(!packet) return OS_FAIL;
    if(os_udp_client_get_lidar_packet(sockfd, packet) == OS_FAIL){
        os_delete_lidar_packet(packet);
        return OS_FAIL;
    }
    os_batch_packet_to_scan(packet, scan);
    scan->frame_id = packet->packet_header.frame_id;
    uint32_t m_id = packet->column_data[0].column_header_block.measurement_id;
    while(1){
        if(os_udp_client_get_lidar_packet(sockfd, packet) == OS_FAIL){
            os_delete_lidar_packet(packet);
            return OS_FAIL;
        }
        if(packet->column_data[0].column_header_block.measurement_id == m_id){
            os_delete_lidar_packet(packet);
            break;
        }
        os_batch_packet_to_scan(packet, scan);
    }
    return OS_SUCCESS;
}

lidar_scan_t* os_alloc_lidar_scan()
{
    lidar_scan_t *scan = (lidar_scan_t *)malloc(sizeof(lidar_scan_t));
    if(!scan) return NULL;
    scan->timestamp = NULL;
    scan->measurement_id = NULL;
    scan->status = NULL;
    scan->rng = NULL;
    scan->ref = NULL;
    scan->sig = NULL;
    scan->nir = NULL;
    scan->rng2 = NULL;
    scan->ref2 = NULL;
    scan->sig2 = NULL;

    scan->w = sensor_info.columns_per_frame;
    scan->h = sensor_info.pixels_per_column;
    scan->frame_id = 0;

    do{
        scan->timestamp = (uint64_t *)malloc(sizeof(uint64_t) * scan->w);
        if(!scan->timestamp) break;
        scan->measurement_id = (uint16_t *)malloc(sizeof(uint16_t) * scan->w);
        if(!scan->measurement_id) break;
        scan->status = (uint32_t *)malloc(sizeof(uint32_t) * scan->w);
        if(!scan->status) break;
        size_t pixels_num = scan->w * scan->h;
        scan->rng = (uint32_t *)malloc(sizeof(uint32_t) * pixels_num);
        if(!scan->rng) break;
        scan->ref = (uint8_t *)malloc(sizeof(uint8_t) * pixels_num);
        if(!scan->ref) break;
        scan->nir = (uint16_t *)malloc(sizeof(uint16_t) * pixels_num);
        if(!scan->nir) break;
        if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16){
            scan->sig = (uint16_t *)malloc(sizeof(uint16_t) * pixels_num);
            if(!scan->sig) break;
        } else if(sensor_info.udp_profile_lidar == RNG19_RFL8_SIG16_NIR16_DUAL){
            scan->sig = (uint16_t *)malloc(sizeof(uint16_t) * pixels_num);
            if(!scan->sig) break;
            scan->rng2 = (uint32_t *)malloc(sizeof(uint32_t) * pixels_num);
            if(!scan->rng2) break;
            scan->ref2 = (uint8_t *)malloc(sizeof(uint8_t) * pixels_num);
            if(!scan->ref2) break;
            scan->sig2 = (uint16_t *)malloc(sizeof(uint16_t) * pixels_num);
            if(!scan->sig2) break;
        }
        memset((void *)scan->timestamp, 0, sizeof(uint64_t) * scan->w);
        memset((void *)scan->measurement_id, 0, sizeof(uint16_t) * scan->w);
        memset((void *)scan->status, 0, sizeof(uint32_t) * scan->w);
        memset((void *)scan->rng, 0, sizeof(uint32_t) * pixels_num);
        memset((void *)scan->ref, 0, sizeof(uint8_t) * pixels_num);
        memset((void *)scan->nir, 0, sizeof(uint16_t) * pixels_num);
        if(scan->sig) memset((void *)scan->sig, 0, sizeof(uint16_t) * pixels_num);
        if(scan->rng2) memset((void *)scan->rng2, 0, sizeof(uint32_t) * pixels_num);
        if(scan->ref2) memset((void *)scan->ref2, 0, sizeof(uint8_t) * pixels_num);
        if(scan->sig2) memset((void *)scan->sig2, 0, sizeof(uint16_t) * pixels_num);
        return scan;
    } while(0);

    if(scan->timestamp) free(scan->timestamp);
    if(scan->measurement_id) free(scan->measurement_id);
    if(scan->status) free(scan->status);
    if(scan->rng) free(scan->rng);
    if(scan->ref) free(scan->ref);
    if(scan->sig) free(scan->sig);
    if(scan->nir) free(scan->nir);
    if(scan->rng2) free(scan->rng2);
    if(scan->ref2) free(scan->ref2);
    if(scan->sig2) free(scan->sig2);
    free(scan);
    return NULL;
}

void os_delete_lidar_scan(lidar_scan_t *scan)
{
    free(scan->timestamp);
    free(scan->measurement_id);
    free(scan->status);
    free(scan->rng);
    free(scan->ref);
    free(scan->nir);
    if(scan->sig) free(scan->sig);
    if(scan->rng2) free(scan->rng2);
    if(scan->ref2) free(scan->ref2);
    if(scan->sig2) free(scan->sig2);
    free(scan);
}

xyz_t* os_cartesian(lidar_scan_t *scan, size_t *count)
{
    xyz_t *xyz = (xyz_t *)malloc(scan->w * scan->h * sizeof(xyz_t));
    if(!xyz) return NULL;
    size_t offset_s;
    double rng2;    // r': range_to_beam_origin
    double n;
    double theta_encoder;
    double theta_azimuth;
    double phi;
    uint32_t m_id;
    uint32_t status;
    *count = 0;
    for(size_t i = 0; i < scan->w; i++){
        m_id = scan->measurement_id[i];
        status = scan->status[i];
        for(size_t j = 0; j < scan->h; j++){
            offset_s = scan->w * j + i;
            if(!status || !scan->rng[offset_s]){
                continue;
            }
            n = sqrt(pow(sensor_info.beam_to_lidar_transform[0][3], 2.0) +
                     pow(sensor_info.beam_to_lidar_transform[2][3], 2.0));
            rng2 = (double)scan->rng[offset_s] - n;
            assert(rng2 >= 0);
            theta_encoder = 2 * M_PI * (1.0 - (double)m_id / scan->w);
            theta_azimuth = - 2 * M_PI * sensor_info.beam_azimuth_angles[j] / 360;
            phi = 2 * M_PI * sensor_info.beam_altitude_angles[j] / 360;
            (xyz + *count)->x = rng2 * cos(theta_encoder + theta_azimuth) * cos(phi) +
               sensor_info.beam_to_lidar_transform[0][3] * cos(theta_encoder);
            (xyz + *count)->y = rng2 * sin(theta_encoder + theta_azimuth) * cos(phi) +
               sensor_info.beam_to_lidar_transform[0][3] * sin(theta_encoder);
            (xyz + *count)->z = rng2 * sin(phi) + sensor_info.beam_to_lidar_transform[2][3];
            (*count)++;
        }
    }
    xyz_t *temp = realloc(xyz, *count * sizeof(xyz_t));
    if(!temp){
        free(xyz);
        return NULL;
    }
    return temp;
}

