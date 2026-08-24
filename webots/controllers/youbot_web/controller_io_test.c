#include "controller_io.h"

#include <stdio.h>
#include <string.h>

int main(void) {
  const char *temp_path = "controller_io_test.tmp";
  const char *final_path = "controller_io_test.bmp";
  const unsigned char pixels[] = {
      255, 0, 0,
      0, 255, 0,
  };

  if (write_bmp24(temp_path, pixels, 2, 1) != 0) return 1;
  if (get_file_mtime(temp_path) < 0) return 2;
  if (replace_file(temp_path, final_path) != 0) return 3;

  FILE *file = fopen(final_path, "rb");
  if (!file) return 4;
  unsigned char signature[2] = {0};
  const size_t bytes_read = fread(signature, 1, sizeof(signature), file);
  fclose(file);
  remove(final_path);

  if (bytes_read != 2 || memcmp(signature, "BM", 2) != 0) return 5;
  return 0;
}
