#ifdef __cplusplus
extern "C" {
#endif
  int edcl_main(void);
  int edcl_loadelf(const char *elf);
  void edcl_close(void);
  int edcl_read(uint64_t addr, int bytes, uint8_t *obuf);
  int edcl_write(uint64_t addr, int bytes, uint8_t *ibuf);
  int log_edcl_read(uint64_t addr, int bytes, uint8_t *obuf);
  int log_edcl_write(uint64_t addr, int bytes, uint8_t *ibuf);
  void log_edcl(const char *fmt, ...);
#ifdef __cplusplus
};
#endif
