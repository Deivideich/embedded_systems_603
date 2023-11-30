#include "waypoints.h"
#include <stdio.h>

void init_waypoint_buffer(struct waypoint_buffer * wp_buf) {
    clean(wp_buf);
}

void add_wp(struct waypoint_buffer * wp_buf, float x, float y) {
    if (wp_buf->size < MAX_WP) {
        struct waypoint wp = {x, y};
        wp_buf->wp_buf[wp_buf->size] = wp;
        wp_buf->size++;
        if(wp_buf->size == 2)
            wp_buf->to = 1;
    } else {
        printf("Error: Waypoint buffer is full.\n");
    }
}

void to_next(struct waypoint_buffer * wp_buf) {
    if (wp_buf->to < wp_buf->size - 1) {
        wp_buf->from++;
        wp_buf->to++;
    }
}

void clean(struct waypoint_buffer * wp_buf){
    wp_buf->size = 0;
    wp_buf->from = 0;
    wp_buf->to = 0;
}