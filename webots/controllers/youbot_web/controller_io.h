#ifndef YOUBOT_WEB_CONTROLLER_IO_H
#define YOUBOT_WEB_CONTROLLER_IO_H

long long get_file_mtime(const char *path);
int replace_file(const char *from_path, const char *to_path);
int write_bmp24(const char *path, const unsigned char *rgb, int width, int height);

#endif
