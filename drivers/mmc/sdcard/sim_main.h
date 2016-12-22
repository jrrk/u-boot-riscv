#ifdef __cplusplus
extern "C" {
#endif
void verilator_main(int argc, char **argv, char **env);
void verilator_loop(unsigned setting, unsigned start, unsigned cmd, unsigned timeout, unsigned arg,
		    unsigned data_start, unsigned blksiz, unsigned blkcnt, unsigned align,
		    unsigned *finish_cmd, unsigned *finish_data, unsigned *crc_ok, unsigned *index_ok, unsigned *transf_cnt_o,
		    unsigned read_buf[], unsigned write_buf[], unsigned response[]);
void verilator_stop(void);
int verilator_finish(void);
void verilator_eval(void);
  void verilator_printf(const char *fmt, ...);
#ifdef __cplusplus
};
#endif
