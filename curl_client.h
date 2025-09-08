#ifndef CURL_CLIENT_H
#define CURL_CLIENT_H

#include <stddef.h>
#include <stdlib.h>
#include <curl/curl.h>

struct memory{
    char *response;
    size_t size;
};

CURL *os_init_curl_client();
void os_deinit_curl_client(CURL *curl);
CURLcode os_curl_get(CURL *curl, char *url, struct memory *mem);
CURLcode os_curl_post(CURL *curl, char *url, char *str);
CURLcode os_curl_put(CURL *curl, char *url, char *str);
CURLcode os_curl_delete(CURL *curl, char *url);

#endif

