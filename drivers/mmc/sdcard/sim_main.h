#ifdef __cplusplus
extern "C" {
#endif
void verilator_main(int argc, char **argv, char **env);
void verilator_loop(unsigned setting, unsigned start, unsigned cmd, unsigned xmit, unsigned arg, unsigned *finish, unsigned *crc_ok, unsigned *index_ok, unsigned response[]);
void verilator_stop(void);
int verilator_finish(void);
void verilator_eval(void);
  void verilator_printf(const char *fmt, ...);
#ifdef __cplusplus
};
#endif
