Vusim_top: Vusimv_top__ALL.a
	g++ -g obj_dir/sim_main.o obj_dir/stub_main.o obj_dir/verilated.o obj_dir/Vusimv_top__ALL.a -o $@ -lm -lstdc++

Vusimv_top__ALL.a: obj_dir/Vusimv_top.mk
	make -C obj_dir -f Vusimv_top.mk CXXFLAGS="-g"

obj_dir/Vusimv_top.mk: usimv_top.sv sd_verilator_model.v sd_crc_16.v sd_crc_7.v sd_cmd_serial_host.v sim_main.cpp
	 verilator -cc usimv_top.sv sd_verilator_model.v sd_crc_16.v sd_crc_7.v sd_cmd_serial_host.v -exe sim_main.cpp stub_main.cpp

clean:
	rm -rf Vusim_top obj_dir

simv: usimv_top.sv sd_verilator_model.v sd_crc_16.v sd_crc_7.v sd_cmd_serial_host.v usimv_top_include.v
	vcs -full64 +lint=TFIPC-L -gui -o $@ usimv_top_template.sv usimv_top.sv sd_verilator_model.v sd_crc_16.v sd_crc_7.v sd_cmd_serial_host.v sd_data_serial_host.v

standalone:
	g++ ../{mmc,mmc-uclass,mmc_write,minion_sdhci}.o obj_dir/*.o -lstdc++ -lc |& cut -d: -f3|sort -u
