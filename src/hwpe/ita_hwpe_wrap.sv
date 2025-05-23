// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Gamze Islamoglu <gislamoglu@iis.ee.ethz.ch>

`include "hci_helpers.svh"

import ita_hwpe_package::*;
import hwpe_ctrl_package::*;
import hwpe_stream_package::*;
import hci_package::*;

module ita_hwpe_wrap
#(
  // hwpe params
  parameter int unsigned AccDataWidth = 1024,
  parameter int unsigned IdWidth      = ID_WIDTH,
  // system params
  parameter int unsigned MemDataWidth = 64,
  parameter int unsigned MP           = (AccDataWidth / MemDataWidth)
) (
  // global signals
  input  logic                      clk_i         ,
  input  logic                      rst_ni        ,
  input  logic                      test_mode_i   ,

  // events
  output logic [N_CORES-1:0][1:0]   evt_o         ,
  output logic                      busy_o        ,

  // tcdm master ports
  output logic [      MP-1:0]                     tcdm_req_o      ,
  input  logic [      MP-1:0]                     tcdm_gnt_i      ,
  output logic [      MP-1:0][31:0]               tcdm_add_o      ,
  output logic [      MP-1:0]                     tcdm_wen_o      ,
  output logic [      MP-1:0][MemDataWidth/8-1:0] tcdm_be_o       ,
  output logic [      MP-1:0][MemDataWidth-1:0]   tcdm_data_o     ,
  input  logic [      MP-1:0][MemDataWidth-1:0]   tcdm_r_data_i   ,
  input  logic [      MP-1:0]                     tcdm_r_valid_i  ,

  // periph slave port
  input  logic                      periph_req_i    ,
  output logic                      periph_gnt_o    ,
  input  logic [        31:0]       periph_add_i    ,
  input  logic                      periph_wen_i    ,
  input  logic [         3:0]       periph_be_i     ,
  input  logic [        31:0]       periph_data_i   ,
  input  logic [ IdWidth-1:0]       periph_id_i     ,
  output logic [        31:0]       periph_r_data_o ,
  output logic                      periph_r_valid_o,
  output logic [ IdWidth-1:0]       periph_r_id_o
);

  localparam hci_size_parameter_t `HCI_SIZE_PARAM(tcdm) = '{
    DW:  AccDataWidth,
    AW:  DEFAULT_AW,
    BW:  DEFAULT_BW,
    UW:  DEFAULT_UW,
    IW:  ID_WIDTH,
    EW:  DEFAULT_EW,
    EHW: DEFAULT_EHW
  };
  `HCI_INTF(tcdm, clk_i);

  hwpe_ctrl_intf_periph #(.ID_WIDTH(IdWidth)) periph (.clk(clk_i));

  assign tcdm.r_user   = '0;
  assign tcdm.r_id     = '0;
  assign tcdm.r_opc    = '0;
  assign tcdm.r_ecc    = '0;
  assign tcdm.egnt     = '0;
  assign tcdm.r_evalid = '0;

  always_comb begin
    periph.req       = periph_req_i;
    periph.add       = periph_add_i;
    periph.wen       = periph_wen_i;
    periph.be        = periph_be_i;
    periph.data      = periph_data_i;
    periph.id        = periph_id_i;
    periph_gnt_o     = periph.gnt;
    periph_r_data_o  = periph.r_data;
    periph_r_valid_o = periph.r_valid;
    periph_r_id_o    = periph.r_id;
  end

  ita_hwpe_top i_ita (
    .clk_i,
    .rst_ni,
    .test_mode_i (test_mode_i ),
    .evt_o       (evt_o       ),
    .busy_o      (busy_o      ),
    .tcdm        (tcdm        ),
    .periph      (periph      )
  );

  // Buffer to synchronize the tcdm_req_o and tcdm_gnt_i signals
  localparam int unsigned BufferDepth = 8;

  typedef struct packed {
    logic [31:0]               add;
    logic                      wen;
    logic [MemDataWidth/8-1:0] be;
    logic [MemDataWidth-1:0]   data;
  } buf_req_t;

  typedef struct packed {
    logic [MemDataWidth-1:0]   data;
  } buf_rsp_t;

  logic [MP-1:0] buf_req_full, buf_req_empty;
  logic [MP-1:0] buf_rsp_full, buf_rsp_empty;
  logic [MP-1:0] buf_req_push, buf_req_pop;
  logic [MP-1:0] buf_rsp_push,  buf_rsp_pop;
  logic [MP-1:0][$clog2(BufferDepth):0] buf_req_usage, buf_rsp_usage;
  buf_req_t [MP-1:0] buf_req_data_in;
  buf_req_t [MP-1:0] buf_req_data_out;
  buf_rsp_t [MP-1:0] buf_rsp_data_out;
  buf_rsp_t [MP-1:0] buf_rsp_data_in;

  logic all_not_full;

  always_comb begin
    // Default values
    buf_req_push = '0;
    buf_req_pop  = '0;
    buf_rsp_push = '0;
    buf_rsp_pop  = '0;

    // Default values for tcdm signals
    tcdm.gnt = 1'b0;
    tcdm.r_valid = 1'b0;
    tcdm.r_data  = '0;

    all_not_full = 1'b1;

    for (int i = 0; i < MP; i++) begin
      if (buf_rsp_usage[i] == BufferDepth) begin
        all_not_full = 1'b0;
      end
    end
    if (tcdm.req && &(buf_req_full == 1'b0) && all_not_full) begin
      tcdm.gnt = 1'b1;
      for (int i = 0; i < MP; i++) begin
        buf_req_push[i] = 1'b1;
        buf_req_data_in[i].add  = tcdm.add + i*(MemDataWidth/8);
        buf_req_data_in[i].wen  = tcdm.wen;
        buf_req_data_in[i].be   = tcdm.be[i*(MemDataWidth/8)+:(MemDataWidth/8)];
        buf_req_data_in[i].data = tcdm.data[i*MemDataWidth+:MemDataWidth];
      end
    end

    for (int i = 0; i < MP; i++) begin
      tcdm_req_o[i]  = !buf_req_empty[i];
      tcdm_add_o[i]  = buf_req_data_out[i].add;
      tcdm_wen_o[i]  = buf_req_data_out[i].wen;
      tcdm_be_o[i]   = buf_req_data_out[i].be;
      tcdm_data_o[i] = buf_req_data_out[i].data;
      if (tcdm_req_o[i] && tcdm_gnt_i[i]) begin
        buf_req_pop[i] = 1'b1;
      end
    end

    for (int i = 0; i < MP; i++) begin
      buf_rsp_push[i] = tcdm_r_valid_i[i];
      buf_rsp_data_in[i].data = tcdm_r_data_i[i];
    end

    if (&(buf_rsp_empty == 1'b0)) begin
      tcdm.r_valid = 1'b1;
      tcdm.r_data  = { >> {buf_rsp_data_out} };
      for (int i = 0; i < MP; i++) begin
        buf_rsp_pop[i] = tcdm.r_ready;
      end
    end
  end
  
  // generate fifo's with depth BufferDepth and one for each port
  for (genvar i = 0; i < MP; i++) begin : gen_hwpe_tcdm_fifo
    fifo_v3 #(
      .DATA_WIDTH ( $bits(buf_req_t) ),
      .DEPTH ( BufferDepth )
    ) i_fifo_req (
      .clk_i,
      .rst_ni,
      .flush_i (1'b0),
      .testmode_i (1'b0),
      .full_o (buf_req_full[i]),
      .empty_o (buf_req_empty[i]),
      .usage_o (buf_req_usage[i]),
      // Onehot mask.
      .data_i (buf_req_data_in[i]),
      .push_i (buf_req_push[i]),
      .data_o (buf_req_data_out[i]),
      .pop_i  (buf_req_pop[i])
    );

    fifo_v3 #(
      .DATA_WIDTH ( $bits(buf_rsp_t) ),
      .DEPTH ( BufferDepth+1 )
    ) i_fifo_rsp (
      .clk_i,
      .rst_ni,
      .flush_i (1'b0),
      .testmode_i (1'b0),
      .full_o (buf_rsp_full[i]),
      .empty_o (buf_rsp_empty[i]),
      .usage_o (buf_rsp_usage[i]),
      // Onehot mask.
      .data_i (buf_rsp_data_in[i]),
      .push_i (buf_rsp_push[i]),
      .data_o (buf_rsp_data_out[i]),
      .pop_i  (buf_rsp_pop[i])
    );
  end


endmodule : ita_hwpe_wrap