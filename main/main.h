#pragma once

void mqtt_publish_status(const char *status);
void mqtt_publish_result(const char *json);
void mqtt_send_announce(void);