#include "controller_io.h"

#include <stdio.h>
#ifdef _WIN32
#include <sys/stat.h>
#include <windows.h>
#define STAT_STRUCT struct _stat64
#define STAT_FN _stat64
#else
#include <sys/stat.h>
#define STAT_STRUCT struct stat
#define STAT_FN stat
#endif

long long get_file_mtime(const char *path) {
  STAT_STRUCT file_stat;
  if (!path || STAT_FN(path, &file_stat) != 0) return -1;
  return (long long)file_stat.st_mtime;
}

int replace_file(const char *from_path, const char *to_path) {
  if (!from_path || !to_path) return -1;
#ifdef _WIN32
  return MoveFileExA(from_path, to_path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)
             ? 0
             : -1;
#else
  remove(to_path);
  return rename(from_path, to_path);
#endif
}

static void write_le16(FILE *file, unsigned int value) {
  fputc((int)(value & 0xff), file);
  fputc((int)((value >> 8) & 0xff), file);
}

static void write_le32(FILE *file, unsigned int value) {
  fputc((int)(value & 0xff), file);
  fputc((int)((value >> 8) & 0xff), file);
  fputc((int)((value >> 16) & 0xff), file);
  fputc((int)((value >> 24) & 0xff), file);
}

int write_bmp24(const char *path, const unsigned char *rgb, int width, int height) {
  if (!path || !rgb || width <= 0 || height <= 0) return -1;

  FILE *file = fopen(path, "wb");
  if (!file) return -1;

  const int row_stride = ((width * 3 + 3) / 4) * 4;
  const unsigned int pixel_bytes = (unsigned int)(row_stride * height);
  const unsigned int file_size = 54U + pixel_bytes;
  unsigned char padding[3] = {0, 0, 0};

  fputc('B', file);
  fputc('M', file);
  write_le32(file, file_size);
  write_le16(file, 0);
  write_le16(file, 0);
  write_le32(file, 54);
  write_le32(file, 40);
  write_le32(file, (unsigned int)width);
  write_le32(file, (unsigned int)height);
  write_le16(file, 1);
  write_le16(file, 24);
  write_le32(file, 0);
  write_le32(file, pixel_bytes);
  write_le32(file, 2835);
  write_le32(file, 2835);
  write_le32(file, 0);
  write_le32(file, 0);

  for (int y = height - 1; y >= 0; --y) {
    const unsigned char *row = rgb + y * width * 3;
    for (int x = 0; x < width; ++x) {
      const unsigned char *pixel = row + x * 3;
      fputc(pixel[2], file);
      fputc(pixel[1], file);
      fputc(pixel[0], file);
    }
    fwrite(padding, 1, (size_t)(row_stride - width * 3), file);
  }

  return fclose(file) == 0 ? 0 : -1;
}
