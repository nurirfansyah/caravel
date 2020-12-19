// SPDX-FileCopyrightText: 2020 Astria Nur Irfansyah
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * top_astria.v
 * (adapted from user_proj_example from  caravel repo)
 *
 * Description:
 * Test circuits containing:
 * 1. Array of synthesized analog comparators for stochastic ADC (3 banks)
 * 2. Support circuits 
 * 3. LIF Neuron (not implemented in this version)
 *
 * (1) Analog Comparator Bank 1, contains 32 comparators
 *      Name  : comp32
 *      Input : vcomp32_a, vcomp32_b --> GPIO analogio (24,25) (offset from dig)
 *      Output: [31:0] comp32out --> GPIO [31:0], 
 *                                   Logic Analyzer (LA) -> [31:0] la_data_out
 *                                   rdata / wbs_dat_o
 * (2) Analog Comparator Bank 2 & 3, contains 256 comparators each
 *      Name  : comp256_1, comp256_2
 *      Input : [1:0] vcomp256_a, [1:0] vcomp256_b --> GPIO analogio (26,27,28,29)
 *      Output: [1:0] comp256out --> GPIO (37,38),
 *                                   Logic Analyzer (LA) -> (32,33)
 *
 *-------------------------------------------------------------
 */

module top_astria #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oen,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7.
    inout [`MPRJ_IO_PADS-8:0] analog_io    
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    wire [31:0] rdata; 
    wire [31:0] wdata;
    wire [BITS-1:0] comp32out;
    wire comp256out;

    wire valid;
    wire [3:0] wstrb;
    wire [31:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = rdata;
    assign wdata = wbs_dat_i;

    // Comparator wires
    //wire [1:0] comp256out;

    // IO
    assign io_out = {comp256out,comp32out[30:0]};   // cut 1 out from comp32
//    assign io_out = {comp256out,comp32out[29:0]};   // cut 2 out from comp32
//    assign io_out = {comp32out[31:0]};   // cut 2 out from comp32
    assign io_oeb = {(`MPRJ_IO_PADS-1){rst}};

    // LA
//    assign la_data_out = {{(127-BITS-2){1'b0}},comp256out,comp32out};
    assign la_data_out = {{(127-BITS){1'b0}},comp32out};

    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oen[65:34] & ~{BITS{valid}};

    // Assuming LA probes [67:66] are for controlling the clk & reset  
    assign clk = (~la_oen[66]) ? la_data_in[66]: wb_clk_i;
    assign rst = (~la_oen[67]) ? la_data_in[67]: wb_rst_i;


    stoch_adc_comp #(
        .BITS(BITS),
        .COMP_TOTAL(128)
    ) stoch_adc_comp(
        .clk(clk),
        .reset(rst),
        .ready(wbs_ack_o),
        .valid(valid),
        .rdata(rdata),
        .wdata(wbs_dat_i),
        .wstrb(wstrb),
        .la_write(la_write),
        .la_input(la_data_in[65:34]),
        .vcomp32_a(analog_io[24]),
        .vcomp32_b(analog_io[25]),
        .vcomp256_a(analog_io[26]),
        .vcomp256_b(analog_io[28]),
//        .vcomp256_a(analog_io[27:26]),
//        .vcomp256_b(analog_io[29:28]),
        .comp32out(comp32out),
        .comp256out(comp256out)
    );

endmodule

module stoch_adc_comp #(
    parameter BITS = 32,
    parameter COMP_TOTAL = 128
)(
    input clk,
    input reset,
    input valid,
    input [3:0] wstrb,
    input [BITS-1:0] wdata,
    input [BITS-1:0] la_write,
    input [BITS-1:0] la_input,
    inout vcomp32_a,
    inout vcomp32_b,
    inout vcomp256_a,
    inout vcomp256_b,
//    input [1:0] vcomp256_a,
//    input [1:0] vcomp256_b,
    output ready,
    output [BITS-1:0] rdata,
    output [BITS-1:0] comp32out,
    output comp256out
//    output [1:0] comp256out
);
    reg ready;
    reg [BITS-1:0] rdata;

    // Comparator output registers
    reg [BITS-1:0] comp32out;    // Bank 1
    reg [COMP_TOTAL-1:0] comp256out1_reg; // Bank 2
//    reg [COMP_TOTAL-1:0] comp256out2_reg; // Bank 3
    wire [COMP_TOTAL-1:0] comp256out1_wire; // Bank 2
//    wire [COMP_TOTAL-1:0] comp256out2_wire; // Bank 3

    // Comparator output shift registers
    reg [COMP_TOTAL-1:0] comp256out1_sreg; // Bank 2
 //   reg [COMP_TOTAL-1:0] comp256out2_sreg; // Bank 3
    reg [7:0] counter_comp_sreg;        // don't forget to adjust according to COMP_TOTAL

    // Take output from LSB of comp output shift reg
//    assign comp256out = comp256out1_wire[0];
    assign comp256out = comp256out1_sreg[0];
//    assign comp256out[0] = comp256out1_sreg[0];
//    assign comp256out[1] = comp256out2_sreg[0];

    // Dummy reg to take write operation from wishbone
    // Maybe useful later.
    reg [31:0] dummy;

    always @(posedge clk) begin
        if (reset) begin
            counter_comp_sreg <= 0;
            ready <= 0;
        end else begin
            ready <= 1'b0;
            
            if (~|la_write) begin
                // shift outputs
                counter_comp_sreg <= counter_comp_sreg + 1;
//                comp256out2_sreg <= {{1'b0},comp256out2_sreg[31:1]};
            end

            if (valid && !ready) begin
                ready <= 1'b1;
                rdata <= comp32out;
                if (wstrb[0]) dummy[7:0]   <= wdata[7:0];
                if (wstrb[1]) dummy[15:8]  <= wdata[15:8];
                if (wstrb[2]) dummy[23:16] <= wdata[23:16];
                if (wstrb[3]) dummy[31:24] <= wdata[31:24];
            end

            if (counter_comp_sreg == 0) begin
                comp256out1_sreg <= comp256out1_reg;
//                comp256out2_sreg <= comp256out2_reg;
            end 
            else begin
                comp256out1_sreg <= {comp256out1_sreg[0],comp256out1_sreg[COMP_TOTAL-1:1]};
            end
        end
    end
/*
    genvar i;
    generate 
        for(i=0; i<BITS; i=i+1) begin
          always @(posedge clk) begin
              if (la_write[i]) count[i] <= la_input[i];
          end
        end
    endgenerate
*/
    genvar j;
    generate 
        for(j=0; j<32; j=j+1) begin
            synthcomp comp32(.clk(clk), .v_a(vcomp32_a), .v_b(vcomp32_b), .comp_out(comp32out[j])); 
        end
    endgenerate

    genvar k;
    generate 
        for(k=0; k<COMP_TOTAL; k=k+1) begin
            synthcomp comp256_1(.clk(clk), .v_a(vcomp256_a), .v_b(vcomp256_b), .comp_out(comp256out1_wire[k])); 
        end    
    endgenerate
/*
    genvar k;
    generate 
        for(k=0; k<COMP_TOTAL; k=k+1) begin
            synthcomp comp256_1(clk, vcomp256_a[0], vcomp256_b[0], comp256out1_wire[k]);
        end
    endgenerate

    genvar l;
    generate 
        for(l=0; l<COMP_TOTAL; l=l+1) begin
            synthcomp comp256_2(clk, vcomp256_a[1], vcomp256_b[1], comp256out2_wire[l]);
        end
    endgenerate

    always @(posedge clk) begin
        comp256out1_reg <= comp256out1_wire;
        comp256out2_reg <= comp256out2_wire;
    end
*/
endmodule

/* ----------------------
Synthesizable analog clocked comparator based on Sky130 NOR4 cells

Similar principle to NAND3 based design reported in:
[1] S. Weaver, B. Hershberg, and U.K. Moon, 
"Digitally Synthesized Stochastic Flash ADC Using Only Standard Digital Cells,"
IEEE Trans. Circuits Syst. I, doi: 10.1109/TCSI.2013.2268571
-------------------------
*/
module synthcomp (
    input clk,
    inout v_a,
    inout v_b,
    output reg comp_out);

wire qa, qb, qx, qcomp_out;

sky130_fd_sc_hd__nor4_1 X_NOR1 (
//    `ifdef USE_POWER_PINS
//        .VPWR(VPWR),
//        .VGND(VGND),
//        .VPB(VPWR),
//        .VNB(VGND),
//    `endif,
    .Y(qa), .A(v_a), .B(qb), .C(qb), .D(clk));
sky130_fd_sc_hd__nor4_1 X_NOR2 (
    .Y(qb), .A(v_b), .B(qa), .C(qa), .D(clk));
sky130_fd_sc_hd__nor4_1 X_NOR3 (
    .Y(qcomp_out), .A(qa), .B(qa), .C(qx), .D(qx));
sky130_fd_sc_hd__nor4_1 X_NOR4 (
    .Y(qx), .A(qb), .B(qb), .C(qcomp_out), .D(qcomp_out));

always @(posedge clk)
begin
    comp_out <= qcomp_out;
end

endmodule

///////////////////////////////////////////////////

// Digital Leaky Integrate & Fire Neuron

//////////////////////////////////////////////////
module lifNeuron(
    input clk,
    input rst,
    input [31:0] In,
    input inhibit,
    output reg signed [31:0] vout= 0,
    output reg spike= 0
    );
    
reg signed [31:0] v= 0;
reg signed [31:0] vth= 32'h0000F000;
reg signed [31:0] add= 0;
reg signed [31:0] m1= 0;
reg signed [31:0] add1= 0;
reg signed [31:0] mul= 0;

always @(posedge clk)
begin
    if(rst)
    begin
        v<= 0;
        mul<= 0;
        add<= 0;
        add1<= 0;
        vout<= 0;
        m1<= 0;
        spike<= 1'b0;
    end
    else if(!rst)
    begin
        if(inhibit)
        begin
            if(add1[31])
            begin
                v<= add1-{add1[31],1'b1,add1[30:1]};
                vout<= add1-{add1[31],1'b1,add1[30:1]};
            end
            else if(!add1[31])
            begin
                v<= add1-{add1[31],1'b0,add1[30:1]};
                vout<= add1-{add1[31],1'b0,add1[30:1]};                
            end
            v <= 0;
            vout <=0;
            spike<= 1'b0;
        end
        else if(!inhibit)
        begin
            if(add1> vth)
            begin
                m1<= 32'h00000000;
                v<= 32'h00000000;
                vout<= 32'h00000000;
                add1<= 32'h00000000;
                add<= 32'h00000000;
                mul<= 32'h00000000;
                spike<= 1'b1;
            end
            else if(add1<= vth)
            begin
                m1<= In;

                add<= m1-v;
                if(add[31])
                begin
                    mul<= {add[31],5'b11111,add[30:5]};    
                end
                else if(!add[31])
                begin
                    mul<= {add[31],5'b00000,add[30:5]};
                end     
                add1<= v+mul;
                v<= add1;
                vout<= add1;
                spike<= 1'b0;
            end
        end
    end
end

endmodule


`default_nettype wire
