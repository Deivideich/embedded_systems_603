#include "waypoints.h"
#include <stdio.h>

void init_waypoint_buffer(struct waypoint_buffer * wp_buf) {
    clean(wp_buf);
}

void add_wp(struct waypoint_buffer * wp_buf, int x, int y) {
    if (wp_buf->size < MAX_WP) {
        struct waypoint wp = {x, y};
        wp_buf->wp_buf[wp_buf->size] = wp;
        wp_buf->size++;
    } else {
        printf("Error: Waypoint buffer is full.\n");
    }
}

void reached(struct waypoint_buffer * wp_buf) {
    if (wp_buf->pointer < wp_buf->size) {
        wp_buf->pointer++;
    } else {
        wp_buf->pointer = -1;
    }
}

void clean(struct waypoint_buffer * wp_buf){
    wp_buf->size = 0;
    wp_buf->pointer = 0;
}