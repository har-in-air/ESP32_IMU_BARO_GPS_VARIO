#ifndef ASYNC_SERVER_H_
#define ASYNC_SERVER_H_

#include <ESPAsyncWebServer.h>

extern AsyncWebServer *server;

void server_init();
String server_ui_size(const size_t bytes);

#endif
