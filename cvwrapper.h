#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

void create_window();
void destroy_all_windows();
int wait_key(int delay);
void display_image_8u(void *buf, size_t width, size_t height);
void display_image_16u(uint16_t *buf, size_t width, size_t height);
void display_image_32u(uint32_t *buf, size_t width, size_t height);

#ifdef __cplusplus
}
#endif

