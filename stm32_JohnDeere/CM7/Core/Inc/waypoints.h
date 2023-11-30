#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#define MAX_WP 50

struct waypoint {
    float x;
    float y;
};

struct waypoint_buffer {
    int size;
    struct waypoint wp_buf[MAX_WP];
    int from;
    int to;
};

void init_waypoint_buffer(struct waypoint_buffer * wp_buf);
void add_wp(struct waypoint_buffer * wp_buf, float x, float y);
void to_next(struct waypoint_buffer * wp_buf);
void clean(struct waypoint_buffer * wp_buf);

#endif /* WAYPOINTS_H */