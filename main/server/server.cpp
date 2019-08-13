#include "Arduino.h"
#include <ESP32WebServer.h>    // https://github.com/Pedroalbuquerque/ESP32WebServer
#include "server.h"

// top level page handling, css and concept adapted from
// https://github.com/G6EJD/ESP32-ESP8266-File-Download-Upload-Delete-Stream-and-Directory

extern "C" {
#include "common.h"
#include "config.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "spiffs_vfs.h"
#include "flashlog.h"
#include "spiflash.h"
#include "options.h"
}

#define TAG "server"

String WebPage = "";

extern ESP32WebServer* pServer;

static void server_appendHeader();
static void server_appendFooter();
static void server_sendHTMLStop();
static void server_sendHTMLContent();
static void server_sendHTMLHeader();
static void server_reportCouldNotCreateFile(String target);
static void server_reportFileNotFound(String target);
static void server_selectInput(String heading1, String command, String arg_calling_name);

static void server_spiffsFileDownload(String filename);
static void server_spiffsFileDelete(String filename);
static int server_fileSize(const char* fname);
static String server_fileSizeToString(int bytes);


void server_homePage(){
   server_sendHTMLHeader();
   WebPage += F("<a href='/download'><button>Download</button></a>");
   WebPage += F("<a href='/upload'><button>Upload</button></a>");
   WebPage += F("<a href='/delete'><button>Delete</button></a>");
   WebPage += F("<a href='/dir'><button>Directory</button></a>");
   server_appendFooter();
   server_sendHTMLContent();
   server_sendHTMLStop(); // Stop is needed because no content length was sent
   }


void server_listDir(){
   DIR *dir = NULL;
   struct dirent *ent;
   char type;
   char size[9];
   char tpath[100];
   struct stat sb;
   int statok;
#ifdef WEBCFG_DEBUG
   ESP_LOGI(TAG,"LIST of DIR [/spiffs/]\r\n");
#endif
   dir = opendir("/spiffs/");
   if (dir) {
      server_sendHTMLHeader();
      WebPage += F("<center><h3 class='rcorners_m'>Directory Contents</h3></center><br>");
      WebPage += F("<table align='center'>");
      WebPage += F("<tr><th>Name</th><th>File Size</th></tr>");
      server_sendHTMLContent();
      while ((ent = readdir(dir)) != NULL) {
         if (WebPage.length() > 1000) {
            server_sendHTMLContent();
            }
    	   sprintf(tpath, "/spiffs/");
         strcat(tpath, ent->d_name);
		   statok = stat(tpath, &sb);
			if (ent->d_type == DT_REG) {
				type = 'f';
				if (statok) {
               strcpy(size, "       ?");
               }
				else {
					if (sb.st_size < (1024*1024)) sprintf(size,"%8d", (int)sb.st_size);
					else if ((sb.st_size/1024) < (1024*1024)) sprintf(size,"%6dKB", (int)(sb.st_size / 1024));
					else sprintf(size,"%6dMB", (int)(sb.st_size / (1024 * 1024)));
				   }
			   }
			else {
				type = 'd';
				strcpy(size, "       -");
			   }
#ifdef WEBCFG_DEBUG
		   ESP_LOGI(TAG,"%c  %s  %s\r\n",type,size,ent->d_name);
#endif
         WebPage += F("<tr><td>");
         if (type == 'd') WebPage += F("(D) "); else WebPage += F("(F) ");
         WebPage += String(ent->d_name);
         WebPage += F("</td><td>");
         WebPage += String(size);
         WebPage += F("</td></tr>");
         }
      WebPage += F("</table>");
      server_sendHTMLContent();
      }
   else {
#ifdef WEBCFG_DEBUG
      ESP_LOGI(TAG,"Error opening directory");
#endif
      server_sendHTMLHeader();
      WebPage += F("<h3>No Files Found</h3>");
      }
   closedir(dir);
   server_appendFooter();
   server_sendHTMLContent();
   server_sendHTMLStop();   // Stop is needed because no content length was sent
   }  


void server_fileUpload(){
   server_appendHeader();
   WebPage += F("<h3>Select File to Upload</h3>");
   WebPage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
   WebPage += F("<input class='buttons' style='width:40%' type='file' name='fupload' id = 'fupload' value=''><br>");
   WebPage += F("<br><button class='buttons' style='width:10%' type='submit'>Upload File</button><br>");
   WebPage += F("<a href='/'>[Back]</a><br><br>");
   server_appendFooter();
   pServer->send(200, "text/html",WebPage);
   }


FILE* UploadFile;  // handleFileUpload called multiple times, so needs to be global

void server_handleFileUpload() {
   HTTPUpload&  httpupload = pServer->upload();
   if (httpupload.status == UPLOAD_FILE_START)  {
      String filename = httpupload.filename;
      if (!filename.startsWith("/spiffs/")) {
         filename = "/spiffs/"+filename;
         }
#ifdef WEBCFG_DEBUG
      ESP_LOGI(TAG,"Upload File Name: %s", filename.c_str());
#endif
      remove(filename.c_str()); // Remove any previous version 
      UploadFile = fopen(filename.c_str(), "wb");
      if (UploadFile == NULL) {
#ifdef WEBCFG_DEBUG
    	   ESP_LOGI(TAG,"Error opening file %s to write", filename.c_str());
#endif
    	   return;
         }
      filename = String();
      }
   else 
   if (httpupload.status == UPLOAD_FILE_WRITE) {
      if (UploadFile) {
		   int res = fwrite(httpupload.buf, 1, httpupload.currentSize, UploadFile);
		   if (res != httpupload.currentSize) {
#ifdef WEBCFG_DEBUG
	    	   ESP_LOGI(TAG,"Error writing to file %d <> %d", res, httpupload.currentSize);
#endif
		      }
         } 
      }
   else 
   if (httpupload.status == UPLOAD_FILE_END){
      if (UploadFile)  {                                    
         fclose(UploadFile);
#ifdef WEBCFG_DEBUG
         ESP_LOGI(TAG,"Upload Size: %d",httpupload.totalSize);
#endif
         WebPage = "";
         server_appendHeader();
         WebPage += F("<h3>File successfully uploaded</h3>");
         WebPage += F("<h2>Uploaded File Name: "); WebPage += httpupload.filename+"</h2>";
         WebPage += F("<h2>File Size: "); WebPage += server_fileSizeToString(httpupload.totalSize) + "</h2><br>";
         server_appendFooter();
         pServer->send(200,"text/html",WebPage);
         } 
      else {
         server_reportCouldNotCreateFile("upload");
         }
      }
   }


// This gets called twice, the first pass selects the input, the second
// pass then processes the command line arguments

void server_fileDownload(){  
   if (pServer->args() > 0 ) {
      if (pServer->hasArg("download")) {
         server_spiffsFileDownload(pServer->arg(0));
         }
      }
   else {
      server_selectInput("Enter filename to download","download","download");
      }
   }



void server_downloadDataLog() {
   if (FlashLogFreeAddress) {
      pServer->sendHeader("Content-Type", "application/octet-stream");
      pServer->sendHeader("Content-Disposition", "attachment; filename=datalog");
      pServer->sendHeader("Connection", "close");
      pServer->setContentLength(FlashLogFreeAddress);
      pServer->send(200, "application/octet-stream", "");

	   uint32_t flashAddr = 0;
      uint8_t buffer[256];
      int bytesRemaining = (int)(FlashLogFreeAddress - flashAddr);
      do {
         int numXmitBytes =  bytesRemaining > 256 ? 256 : bytesRemaining;  
		   spiflash_readBuffer(flashAddr, buffer, numXmitBytes);
         pServer->sendContent_P((const char*)buffer, numXmitBytes);
         flashAddr += numXmitBytes;
         bytesRemaining = (int)(FlashLogFreeAddress - flashAddr);
         delayMs(10);
		   } while (bytesRemaining >= 0);
	   }
   else {
      server_reportFileNotFound("datalog"); 
      }
   }


void server_fileDelete(){
   if (pServer->args() > 0 ) {
      if (pServer->hasArg("delete")) {
         server_spiffsFileDelete(pServer->arg(0));
         }
      }
   else {
      server_selectInput("Select file to delete","delete","delete");
      }
   }


////////////////////////////////////////////////////////////////////////////////////

static void server_appendHeader() {
   WebPage  = F("<!DOCTYPE html><html>");
   WebPage += F("<head>");
   WebPage += F("<title>ESP32 Gps Vario</title>"); // NOTE: 1em = 16px
   WebPage += F("<meta name='viewport' content='user-scalable=yes,initial-scale=1.0,width=device-width'>");
   WebPage += F("<style>");
   WebPage += F("body{max-width:65%;margin:0 auto;font-family:arial;font-size:105%;text-align:center;color:blue;background-color:#F7F2Fd;}");
   WebPage += F("ul{list-style-type:none;margin:0.1em;padding:0;border-radius:0.375em;overflow:hidden;background-color:#b79e64;font-size:1em;}");
   WebPage += F("li{float:left;border-radius:0.375em;border-right:0.06em solid #bbb;}last-child {border-right:none;font-size:85%}");
   WebPage += F("li a{display: block;border-radius:0.375em;padding:0.44em 0.44em;text-decoration:none;font-size:85%}");
   WebPage += F("li a:hover{background-color:#f2cf7d;border-radius:0.375em;font-size:85%}");
   WebPage += F("section {font-size:0.88em;}");
   WebPage += F("h1{color:white;border-radius:0.5em;font-size:1em;padding:0.2em 0.2em;background:#558ED5;}");
   WebPage += F("h2{color:orange;font-size:1.0em;}");
   WebPage += F("h3{font-size:0.8em;}");
   WebPage += F("table{font-family:arial,sans-serif;font-size:0.9em;border-collapse:collapse;width:85%;}");
   WebPage += F("th,td {border:0.06em solid #dddddd;text-align:left;padding:0.3em;border-bottom:0.06em solid #dddddd;}");
   WebPage += F("tr:nth-child(odd) {background-color:#eeeeee;}");
   WebPage += F(".rcorners_n {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:20%;color:white;font-size:75%;}");
   WebPage += F(".rcorners_m {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:50%;color:white;font-size:75%;}");
   WebPage += F(".rcorners_w {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:70%;color:white;font-size:75%;}");
   WebPage += F(".column{float:left;width:50%;height:45%;}");
   WebPage += F(".row:after{content:'';display:table;clear:both;}");
   WebPage += F("*{box-sizing:border-box;}");
   WebPage += F("footer{background-color:#dddd80; text-align:center;padding:0.3em 0.3em;border-radius:0.375em;font-size:60%;}");
   WebPage += F("button{border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:20%;color:white;font-size:130%;}");
   WebPage += F(".buttons {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:15%;color:white;font-size:80%;}");
   WebPage += F(".buttonsm{border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:9%; color:white;font-size:70%;}");
   WebPage += F(".buttonm {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:15%;color:white;font-size:70%;}");
   WebPage += F(".buttonw {border-radius:0.5em;background:#558ED5;padding:0.3em 0.3em;width:40%;color:white;font-size:70%;}");
   WebPage += F("a{font-size:75%;}");
   WebPage += F("p{font-size:75%;}");
   WebPage += F("</style></head><body><h1>ESP32 Gps Vario </h1>");
   }


static void server_appendFooter(){ 
   WebPage += F("<ul>");
   WebPage += F("<li><a href='/'>Home</a></li>"); // Lower Menu bar command entries
   WebPage += F("<li><a href='/download'>Download</a></li>");
   WebPage += F("<li><a href='/upload'>Upload</a></li>");
   WebPage += F("<li><a href='/delete'>Delete</a></li>");
   WebPage += F("<li><a href='/dir'>Directory</a></li>");
   WebPage += F("</ul>");
   WebPage += F("</body></html>");
   }


static void server_sendHTMLHeader(){
   pServer->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
   pServer->sendHeader("Pragma", "no-cache");
   pServer->sendHeader("Expires", "-1");
   pServer->setContentLength(CONTENT_LENGTH_UNKNOWN);
   pServer->send(200, "text/html", ""); // no Content-length header so we have to close the socket ourselves.
   server_appendHeader();
   pServer->sendContent(WebPage);
   WebPage = "";
   }


static void server_sendHTMLContent(){
   pServer->sendContent(WebPage);
   WebPage = "";
   }


static void server_sendHTMLStop(){
   pServer->sendContent("");
   pServer->client().stop(); // Stop is needed because no content length was sent
   }



static void server_reportFileNotFound(String target){
   server_sendHTMLHeader();
   WebPage += F("<h3>File does not exist</h3>");
   WebPage += F("<a href='/");
   WebPage += target + "'>[Back]</a><br><br>";
   server_appendFooter();
   server_sendHTMLContent();
   server_sendHTMLStop();
   }


static void server_reportCouldNotCreateFile(String target){
   server_sendHTMLHeader();
   WebPage += F("<h3>Could Not Create Uploaded File (write-protected?)</h3>");
   WebPage += F("<a href='/");
   WebPage += target + "'>[Back]</a><br><br>";
   server_appendFooter();
   server_sendHTMLContent();
   server_sendHTMLStop();
   }


static void server_selectInput(String heading1, String command, String arg_calling_name){
   server_sendHTMLHeader();
   WebPage += F("<h3>");
   WebPage += heading1 + "</h3>";
   WebPage += F("<FORM action='/");
   WebPage += command + "' method='post'>"; // Must match the calling argument
   WebPage += F("<input type='text' name='");
   WebPage += arg_calling_name;
   WebPage += F("' value=''><br>");
   WebPage += F("<type='submit' name='");
   WebPage += arg_calling_name;
   WebPage += F("' value=''><br><br>");
   server_appendFooter();
   server_sendHTMLContent();
   server_sendHTMLStop();
   }


static int server_fileSize(const char* fname) {
   struct stat sb;
   int statok;
	statok = stat(fname, &sb);
   return (statok == 0) ? sb.st_size : -1;
   }


static String server_fileSizeToString(int bytes){
   String sz = "";
   if (bytes < 0)                    sz = "-?-";
   if (bytes < 1024)                 sz = String(bytes)+" B";
   else if(bytes < (1024*1024))      sz = String(bytes/1024.0,3)+" KB";
   return sz;
   }



static void server_spiffsFileDownload(String filename){
   String  fsname = "/spiffs/"+filename;
   FILE* fdDownload = fopen(fsname.c_str(), "r");
   if (fdDownload) {
      pServer->sendHeader("Content-Type", "text/text");
      pServer->sendHeader("Content-Disposition", "attachment; filename=" + filename);
      pServer->sendHeader("Connection", "close");
      int sizeBytes = server_fileSize(fsname.c_str());
      if (sizeBytes < 0) {
#ifdef WEBCFG_DEBUG
         ESP_LOGI(TAG,"Error file %s size bytes = -1", fsname.c_str());
#endif
         } 
      pServer->setContentLength(sizeBytes);
      pServer->send(200, "application/octet-stream", "");
      uint8_t buf[256];
      int numBytesToSend = sizeBytes;
      while(numBytesToSend > 0){
         int bytesRead = fread(buf, 1,256, fdDownload);
         pServer->sendContent_P((const char*)buf, bytesRead);
         numBytesToSend -= bytesRead;
         delayMs(10);
         }
      fclose(fdDownload);
      } 
   else {
      server_reportFileNotFound("download"); 
      }
   }



static void server_spiffsFileDelete(String filename) {
   server_sendHTMLHeader();
   String fsname = "/spiffs/" + filename;
   FILE* fd = fopen(fsname.c_str(), "r"); 
   if (fd)  {
      fclose(fd);
      remove(fsname.c_str());
#ifdef WEBCFG_DEBUG
      ESP_LOGI(TAG,"File deleted successfully");
#endif
      WebPage += "<h3>File '" + filename + "' has been erased</h3>";
      WebPage += F("<a href='/delete'>[Back]</a><br><br>");
      } 
   else {
      server_reportFileNotFound("delete");
      }
   server_appendFooter(); 
   server_sendHTMLContent();
   server_sendHTMLStop();
   }

