// 
// (c) Copyright 2008 - 2013 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
// 

// Copyright 2015 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
// See LICENSE for license details.

`default_nettype none

module usimv_top(
 input 		 sd_clk,
 input 		 rst,
 input [2:0] 	 setting_i,
 input 		 start_i,
 input [31:0] 	 arg_i,
 input [5:0] 	 cmd_i,
 input [31:0] 	 timeout_i,
 input [2:0] 	 sd_data_start_i,
 input [1:0] 	 sd_align_i,
 input [15:0] 	 sd_blkcnt_i,
 input [11:0] 	 sd_blksize_i,
 input [31:0] 	 sd_data_i,
//---------------Output ports---------------
 output [31:0] 	 response0_o,
 output [63:32]  response1_o,
 output [95:64]  response2_o,
 output [126:96] response3_o,
 output [31:0] 	 wait_o,
 output [31:0] 	 wait_data_o,
 output [31:0] 	 status_o,
 output [31:0] 	 packet0_o,
 output [15:0] 	 packet1_o,
 output [6:0] 	 crc_val_o,
 output [6:0] 	 crc_actual_o,
 output 	 finish_cmd_o,
 output 	 finish_data_o,
 output 	 crc_ok_o,
 output 	 index_ok_o,
 output 	 sd_rd_o,
 output 	 sd_we_o,
 output [31:0] 	 sd_data_o,
 output [15:0] 	 transf_cnt_o);

   wire [3:0] 	    sd_dat_to_mem;
   wire [3:0] 	    sd_dat_to_host;
   wire 	    sd_cmd_to_mem;
   wire 	    sd_cmd_to_host;
   wire 	    sd_cmd_oe, sd_dat_oe;

   reg 		    sd_cmd_to_host_dly;
   reg [3:0] 	    sd_dat_to_host_dly;
		    
   wire		    start_data;
   wire 	    data_crc_ok;
   wire 	    sd_busy, sd_data_busy;

sd_top sdtop(
    .sd_clk     (sd_clk),
    .cmd_rst    (rst),
    .data_rst   (rst),
    .setting_i  (setting_i),
    .timeout_i  (timeout_i),
    .cmd_i      (cmd_i),
    .arg_i      (arg_i),
    .start_i    (start_i),
    .sd_data_start_i(sd_data_start_i),
    .sd_align_i(sd_align_i),
    .sd_blkcnt_i(sd_blkcnt_i),
    .sd_blksize_i(sd_blksize_i),
    .sd_data_i(sd_data_i),
    .sd_dat_to_host(sd_dat_to_host),
    .sd_cmd_to_host(sd_cmd_to_host),
    .finish_cmd_o(finish_cmd_o),
    .finish_data_o(finish_data_o),
    .response0_o(response0_o),
    .response1_o(response1_o),
    .response2_o(response2_o),
    .response3_o(response3_o),
    .crc_ok_o   (crc_ok_o),
    .index_ok_o (index_ok_o),
    .transf_cnt_o(transf_cnt_o),
    .wait_o(wait_o),
    .wait_data_o(wait_data_o),
    .status_o(status_o[31:4]),
    .packet0_o(packet0_o),
    .packet1_o(packet1_o),
    .crc_val_o(crc_val_o),
    .crc_actual_o(crc_actual_o),
    .sd_rd_o(sd_rd_o),
    .sd_we_o(sd_we_o),
    .sd_data_o(sd_data_o),    
    .sd_dat_to_mem(sd_dat_to_mem),
    .sd_cmd_to_mem(sd_cmd_to_mem),
    .sd_dat_oe(sd_dat_oe),
    .sd_cmd_oe(sd_cmd_oe)
    );

   assign status_o[3:0] = 4'b0000;
   
sd_verilator_model sdflash1 (
             .sdClk(~sd_clk),
             .cmd(sd_cmd_to_mem),
             .cmdOut(sd_cmd_to_host),
             .dat(sd_dat_to_mem),
             .datOut(sd_dat_to_host)
);
  
endmodule // chip_top
`default_nettype wire
