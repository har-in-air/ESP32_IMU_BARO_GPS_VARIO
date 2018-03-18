#ifndef SERVER_H_
#define SERVER_H_

#include "Arduino.h"

void server_homePage();
void server_listDir();
void server_fileUpload();
void server_fileDownload();
void server_fileDelete();
void server_handleFileUpload();
void server_downloadDataLog();

#endif
