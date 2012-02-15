/*
 * Milkyminer
 *
 * Copyright (c) 2011 fpgaminer@bitcoin-mining.com
 * Copyleft 2012 paul@kristianpaul.org
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

`timescale 1ns/1ps

module system(
	input clk50,

	// UART
	input uart_rx,
	output uart_tx,

	// GPIO
//	input btn1,
//	input btn2,
//	input btn3,
	output led1,
	output led2

	// Exp miner
	//output [EXT_PORTS-1:0] extminer_txd,
	//input [EXT_PORTS-1:0]  extminer_rxd,

	// Expansion connector
//	input [11:0] exp
	
	//DIP

);


	// The LOOP_LOG2 parameter determines how unrolled the SHA-256
	// calculations are. For example, a setting of 1 will completely
	// unroll the calculations, resulting in 128 rounds and a large, fast
	// design.
	//
	// A setting of 2 will result in 64 rounds, with half the size and
	// half the speed. 3 will be 32 rounds, with 1/4th the size and speed.
	// And so on.
	//
	// Valid range: [0, 5]
`ifdef CONFIG_LOOP_LOG2
	parameter LOOP_LOG2 = `CONFIG_LOOP_LOG2;
`else
	parameter LOOP_LOG2 = 5;
`endif

	// No need to adjust these parameters
	localparam [5:0] LOOP = (6'd1 << LOOP_LOG2);
	// The nonce will always be larger at the time we discover a valid
	// hash. This is its offset from the nonce that gave rise to the valid
	// hash (except when LOOP_LOG2 == 0 or 1, where the offset is 131 or
	// 66 respectively).
	localparam [31:0] GOLDEN_NONCE_OFFSET = (32'd1 << (7 - LOOP_LOG2)) + 32'd1;

	//// 
	reg [255:0] state = 0;
	reg [511:0] data = 0;
   	reg [31:0] nonce = 32'h00000000;

	//// DCM
	wire hash_clk;
	wire hash_clk_dcm;
//     	50/5*8 = 80Mhz

	DCM_SP #(
	.CLKDV_DIVIDE(2.0),		// 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5

	.CLKFX_DIVIDE(5),		// 1 to 32
	.CLKFX_MULTIPLY(8),		// 2 to 32

	.CLKIN_DIVIDE_BY_2("FALSE"),
	.CLKIN_PERIOD(20.0),
	.CLKOUT_PHASE_SHIFT("NONE"),
	.CLK_FEEDBACK("NONE"),
	.DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
	.DUTY_CYCLE_CORRECTION("TRUE"),
	.PHASE_SHIFT(0),
	.STARTUP_WAIT("TRUE")
) clkgen_hash (
	.CLK0(),
	.CLK90(),
	.CLK180(),
	.CLK270(),

	.CLK2X(),
	.CLK2X180(),

	.CLKDV(),
	.CLKFX(hash_clk_dcm),
	.CLKFX180(),
	.LOCKED(),
	.CLKFB(),
	.CLKIN(clk50),
	.RST(1'b0),
	.PSEN(1'b0)
);
BUFG b1(
	.I(hash_clk_dcm),
	.O(hash_clk)
);

	//// Hashers
	wire [255:0] hash, hash2;
	reg [5:0] cnt = 6'd0;
	reg feedback = 1'b0;

	sha256_transform #(.LOOP(LOOP)) uut (
		.clk(hash_clk),
		.feedback(feedback),
		.cnt(cnt),
		.rx_state(state),
		.rx_input(data),
		.tx_hash(hash)
	);
	sha256_transform #(.LOOP(LOOP)) uut2 (
		.clk(hash_clk),
		.feedback(feedback),
		.cnt(cnt),
		.rx_state(256'h5be0cd191f83d9ab9b05688c510e527fa54ff53a3c6ef372bb67ae856a09e667),
		.rx_input({256'h0000010000000000000000000000000000000000000000000000000080000000, hash}),
		.tx_hash(hash2)
	);


	//// Virtual Wire Control
	reg [255:0] midstate_buf = 0, data_buf = 0;
	wire [255:0] midstate_vw, data2_vw;

   
   serial_receive serrx (.clk(hash_clk), .RxD(uart_rx), .midstate(midstate_vw), .data2(data2_vw));
   
	//// Virtual Wire Output
	reg [31:0] golden_nonce = 0;
   reg 		   serial_send;
   wire 	   serial_busy;

   serial_transmit sertx (.clk(hash_clk), .TxD(uart_tx), .send(serial_send), .busy(serial_busy), .word(golden_nonce));
   

	//// Control Unit
	reg is_golden_ticket = 1'b0;
	reg feedback_d1 = 1'b1;
	wire [5:0] cnt_next;
	wire [31:0] nonce_next;
	wire feedback_next;
	wire reset;
	assign reset = 1'b0;

	assign cnt_next =  reset ? 6'd0 : (LOOP == 1) ? 6'd0 : (cnt + 6'd1) & (LOOP-1);
	// On the first count (cnt==0), load data from previous stage (no feedback)
	// on 1..LOOP-1, take feedback from current stage
	// This reduces the throughput by a factor of (LOOP), but also reduces the design size by the same amount
	assign feedback_next = (LOOP == 1) ? 1'b0 : (cnt_next != {(LOOP_LOG2){1'b0}});
	assign nonce_next =
		reset ? 32'd0 :
		feedback_next ? nonce : (nonce + 32'd1);

	
	always @ (posedge hash_clk)
	begin
		midstate_buf <= midstate_vw;
		data_buf <= data2_vw;

		cnt <= cnt_next;
		feedback <= feedback_next;
		feedback_d1 <= feedback;

		// Give new data to the hasher
		state <= midstate_buf;
		data <= {384'h000002800000000000000000000000000000000000000000000000000000000000000000000000000000000080000000, nonce_next, data_buf[95:0]};
		nonce <= nonce_next;


		// Check to see if the last hash generated is valid.
		is_golden_ticket <= (hash2[255:224] == 32'h00000000) && !feedback_d1;
		if(is_golden_ticket)
		begin
			// TODO: Find a more compact calculation for this
			if (LOOP == 1)
				golden_nonce <= nonce - 32'd131;
			else if (LOOP == 2)
				golden_nonce <= nonce - 32'd66;
			else
				golden_nonce <= nonce - GOLDEN_NONCE_OFFSET;

		   if (!serial_busy) serial_send <= 1;
		end // if (is_golden_ticket)
		else
		serial_send <= 0;

	end

/* debug led serial */
//assign led1 = ~uart_rx;
assign led1 = |golden_nonce;
assign led2 = ~uart_tx;
   
endmodule

