#include "curl_client.h"
#include "string.h"

CURL *os_init_curl_client()
{
    curl_global_init(CURL_GLOBAL_DEFAULT);
    return curl_easy_init();
}

void os_deinit_curl_client(CURL *curl)
{
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

static size_t write_callback(void *buffer, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    struct memory *mem = (struct memory *)userp;
    char *p = realloc(mem->response, mem->size + realsize + 1);
    if(!p) return 0;
    mem->response = p;
    memcpy(&(mem->response[mem->size]), buffer, realsize);
    mem->size += realsize;
    mem->response[mem->size] = '\0';
    return realsize;
}

CURLcode os_curl_get(CURL *curl, char *url, struct memory *mem)
{
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)mem);
    curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
    return curl_easy_perform(curl);
}

CURLcode os_curl_post(CURL *curl, char *url, char *str)
{
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, str);
    CURLcode res;
    res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    return res;
}

CURLcode os_curl_put(CURL *curl, char *url, char *str)
{
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    struct curl_slist *headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "PUT");
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, str);
    CURLcode res;
    res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    return res;
}

CURLcode os_curl_delete(CURL *curl, char *url)
{
    curl_easy_reset(curl);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, "DELETE");
    CURLcode res;
    res = curl_easy_perform(curl);
    return res;
}

