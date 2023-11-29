#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#define MAX_WP 50

struct waypoint {
    int x;
    int y;
};

struct waypoint_buffer {
    int size;
    struct waypoint wp_buf[MAX_WP];
    int pointer;
};

void init_waypoint_buffer(struct waypoint_buffer * wp_buf);
void add_wp(struct waypoint_buffer * wp_buf, int x, int y);
void reached(struct waypoint_buffer * wp_buf);
void clean(struct waypoint_buffer * wp_buf);

#endif /* WAYPOINTS_H */