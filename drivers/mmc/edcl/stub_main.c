#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

enum edcl_mode {
  edcl_mode_unknown,
  edcl_mode_read,
  edcl_mode_write,
  edcl_mode_block_read,
  edcl_mode_bootstrap,
  edcl_max=256};

#pragma pack(4)

static struct etrans {
  enum edcl_mode mode;
  volatile uint32_t *ptr;
  uint32_t val;
} tmp, tmp2;

#pragma pack()

void edcl_bootstrap(int entry)
{
  tmp.val = 0xDEADBEEF;
  tmp.mode = edcl_mode_bootstrap;
  tmp.ptr = (void *)(size_t)entry;
  edcl_write(0, sizeof(struct etrans), (uint8_t*)&tmp);
  edcl_read(0, sizeof(struct etrans), (uint8_t*)&tmp2);
  edcl_write(edcl_max*sizeof(struct etrans), sizeof(struct etrans), (uint8_t*)&tmp);
}

int main(int argc, char* argv[]) {
  edcl_main();

  // Parse arguments:
    if (argc > 1) {
      int i, entry = 0;
        for (i = 1; i < argc; i++) {
	  if (strcmp(argv[i], "-loadelf") == 0) entry=edcl_loadelf(argv[++i]);
	  else if (strcmp(argv[i], "-bootstrap") == 0) edcl_bootstrap(entry);
	  else fprintf(stderr, "Invalid flag %s\n", argv[i]);
        }
    }

    edcl_close();
    return 0;
}
