/*
 * Milkminer SoC (based on Milkymist)
 * Copyright (C) 2007, 2008, 2009, 2010, 2011 Sebastien Bourdeauducq
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

`include "setup.v"
`include "lm32_include.v"

module system(
	input clk50,

	// Boot ROM
	output [23:0] flash_adr,
	inout [15:0] flash_d,
	output flash_oe_n,
	output flash_we_n,
	output flash_ce_n,
	output flash_rst_n,
	input flash_sts,

	// UART
	input uart_rx,
	output uart_tx,

	// GPIO
	input btn1,
	input btn2,
	input btn3,
	output led1,
	output led2,

	// DDR SDRAM
	output sdram_clk_p,
	output sdram_clk_n,
	output sdram_cke,
	output sdram_cs_n,
	output sdram_we_n,
	output sdram_cas_n,
	output sdram_ras_n,
	output [3:0] sdram_dm,
	output [12:0] sdram_adr,
	output [1:0] sdram_ba,
	inout [31:0] sdram_dq,
	inout [3:0] sdram_dqs,

	// Ethernet
	output phy_rst_n,
	input phy_tx_clk,
	output [3:0] phy_tx_data,
	output phy_tx_en,
	output phy_tx_er,
	input phy_rx_clk,
	input [3:0] phy_rx_data,
	input phy_dv,
	input phy_rx_er,
	input phy_col,
	input phy_crs,
	input phy_irq_n,
	output phy_mii_clk,
	inout phy_mii_data,
	output reg phy_clk,

	// PCB revision
	input [3:0] pcb_revision
);

//------------------------------------------------------------------
// Clock and Reset Generation
//------------------------------------------------------------------
wire sys_clk;
wire sys_clk_n;
wire hard_reset;
wire reset_button = btn1 & btn2 & btn3;

`ifndef SIMULATION
wire sys_clk_dcm;
wire sys_clk_n_dcm;

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
) clkgen_sys (
	.CLK0(),
	.CLK90(),
	.CLK180(),
	.CLK270(),

	.CLK2X(),
	.CLK2X180(),

	.CLKDV(),
	.CLKFX(sys_clk_dcm),
	.CLKFX180(sys_clk_n_dcm),
	.LOCKED(),
	.CLKFB(),
	.CLKIN(clk50),
	.RST(1'b0),
	.PSEN(1'b0)
);
BUFG b1(
	.I(sys_clk_dcm),
	.O(sys_clk)
);
BUFG b2(
	.I(sys_clk_n_dcm),
	.O(sys_clk_n)
);
`else
assign sys_clk = clkin;
assign sys_clk_n = ~clkin;
`endif

reg trigger_reset;
always @(posedge sys_clk) trigger_reset <= hard_reset|reset_button;
reg [19:0] rst_debounce;
reg sys_rst;
initial rst_debounce <= 20'hFFFFF;
initial sys_rst <= 1'b1;
always @(posedge sys_clk) begin
	if(trigger_reset)
		rst_debounce <= 20'hFFFFF;
	else if(rst_debounce != 20'd0)
		rst_debounce <= rst_debounce - 20'd1;
	sys_rst <= rst_debounce != 20'd0;
end

assign ac97_rst_n = ~sys_rst;
assign videoin_rst_n = ~sys_rst;

/*
 * We must release the Flash reset before the system reset
 * because the Flash needs some time to come out of reset
 * and the CPU begins fetching instructions from it
 * as soon as the system reset is released.
 * From datasheet, minimum reset pulse width is 100ns
 * and reset-to-read time is 150ns.
 */

reg [7:0] flash_rstcounter;
initial flash_rstcounter <= 8'd0;
always @(posedge sys_clk) begin
	if(trigger_reset)
		flash_rstcounter <= 8'd0;
	else if(~flash_rstcounter[7])
		flash_rstcounter <= flash_rstcounter + 8'd1;
end

assign flash_rst_n = flash_rstcounter[7];

//------------------------------------------------------------------
// Wishbone master wires
//------------------------------------------------------------------
wire [31:0]	cpuibus_adr,
		cpudbus_adr;

wire [2:0]	cpuibus_cti,
		cpudbus_cti;

wire [31:0]	cpuibus_dat_r,
		cpudbus_dat_r,
		cpudbus_dat_w;

wire [3:0]	cpudbus_sel;

wire		cpudbus_we;

wire		cpuibus_cyc,
		cpudbus_cyc;

wire		cpuibus_stb,
		cpudbus_stb;

wire		cpuibus_ack,
		cpudbus_ack;

//------------------------------------------------------------------
// Wishbone slave wires
//------------------------------------------------------------------
wire [31:0]	norflash_adr,
		eth_adr,
		brg_adr,
		csrbrg_adr;

wire [2:0]	brg_cti;

wire [31:0]	norflash_dat_r,
		norflash_dat_w,
		eth_dat_r,
		eth_dat_w,
		csrbrg_dat_r,
		csrbrg_dat_w;

wire [3:0]	norflash_sel,
		eth_sel,
		brg_sel;

wire		norflash_we,
		usb_we,
		eth_we,
		brg_we,
		csrbrg_we;

wire		norflash_cyc,
		usb_cyc,
		eth_cyc,
		brg_cyc,
		csrbrg_cyc;

wire		norflash_stb,
		usb_stb,
		eth_stb,
		brg_stb,
		csrbrg_stb;

wire		norflash_ack,
		usb_ack,
		eth_ack,
		brg_ack,
		csrbrg_ack;

//---------------------------------------------------------------------------
// Wishbone switch
//---------------------------------------------------------------------------
// norflash     0x00000000 (shadow @0x80000000)
// Ethernet     0x30000000 (shadow @0xb0000000)
// SDRAM        0x40000000 (shadow @0xc0000000)
// CSR bridge   0x60000000 (shadow @0xe0000000)

// MSB (Bit 31) is ignored for slave address decoding
conbus5x6 #(
	.s0_addr(3'b000), // norflash
//	.s1_addr(3'b001), // 
//	.s2_addr(3'b010), //
	.s3_addr(3'b011), // Ethernet
	.s4_addr(2'b10),  // SDRAM
	.s5_addr(2'b11)   // CSR
) wbswitch (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	// Master 0
	.m0_dat_i(32'hx),
	.m0_dat_o(cpuibus_dat_r),
	.m0_adr_i(cpuibus_adr),
	.m0_cti_i(cpuibus_cti),
	.m0_we_i(1'b0),
	.m0_sel_i(4'hf),
	.m0_cyc_i(cpuibus_cyc),
	.m0_stb_i(cpuibus_stb),
	.m0_ack_o(cpuibus_ack),
	// Master 1
	.m1_dat_i(cpudbus_dat_w),
	.m1_dat_o(cpudbus_dat_r),
	.m1_adr_i(cpudbus_adr),
	.m1_cti_i(cpudbus_cti),
	.m1_we_i(cpudbus_we),
	.m1_sel_i(cpudbus_sel),
	.m1_cyc_i(cpudbus_cyc),
	.m1_stb_i(cpudbus_stb),
	.m1_ack_o(cpudbus_ack),
	// Master 2
	.m2_dat_i(),
	.m2_dat_o(),
	.m2_adr_i(),
	.m2_cti_i(),
	.m2_we_i(),
	.m2_sel_i(),
	.m2_cyc_i(),
	.m2_stb_i(),
	.m2_ack_o(),
	// Master 3
	.m3_dat_i(),
	.m3_dat_o(),
	.m3_adr_i(),
	.m3_cti_i(),
	.m3_we_i(),
	.m3_sel_i(),
	.m3_cyc_i(),
	.m3_stb_i(),
	.m3_ack_o(),
	// Master 4
	.m4_dat_i(),
	.m4_dat_o(),
	.m4_adr_i(),
	.m4_cti_i(),
	.m4_we_i(),
	.m4_sel_i(),
	.m4_cyc_i(),
	.m4_stb_i(),
	.m4_ack_o(),

	// Slave 0
	.s0_dat_i(norflash_dat_r),
	.s0_dat_o(norflash_dat_w),
	.s0_adr_o(norflash_adr),
	.s0_cti_o(),
	.s0_sel_o(norflash_sel),
	.s0_we_o(norflash_we),
	.s0_cyc_o(norflash_cyc),
	.s0_stb_o(norflash_stb),
	.s0_ack_i(norflash_ack),
	// Slave 1
	.s1_dat_i(),
	.s1_dat_o(),
	.s1_adr_o(),
	.s1_cti_o(),
	.s1_sel_o(),
	.s1_we_o(),
	.s1_cyc_o(),
	.s1_stb_o(),
	.s1_ack_i(),
	// Slave 2
	.s2_dat_i(),
	.s2_dat_o(),
	.s2_adr_o(),
	.s2_cti_o(),
	.s2_sel_o(),
	.s2_we_o(),
	.s2_cyc_o(),
	.s2_stb_o(),
	.s2_ack_i(),
	// Slave 3
	.s3_dat_i(eth_dat_r),
	.s3_dat_o(eth_dat_w),
	.s3_adr_o(eth_adr),
	.s3_cti_o(),
	.s3_sel_o(eth_sel),
	.s3_we_o(eth_we),
	.s3_cyc_o(eth_cyc),
	.s3_stb_o(eth_stb),
	.s3_ack_i(eth_ack),
	// Slave 4
	.s4_dat_i(),
	.s4_dat_o(),
	.s4_adr_o(),
	.s4_cti_o(),
	.s4_sel_o(),
	.s4_we_o(),
	.s4_cyc_o(),
	.s4_stb_o(),
	.s4_ack_i(),
	// Slave 5
	.s5_dat_i(csrbrg_dat_r),
	.s5_dat_o(csrbrg_dat_w),
	.s5_adr_o(csrbrg_adr),
	.s5_cti_o(),
	.s5_sel_o(),
	.s5_we_o(csrbrg_we),
	.s5_cyc_o(csrbrg_cyc),
	.s5_stb_o(csrbrg_stb),
	.s5_ack_i(csrbrg_ack)
);

//------------------------------------------------------------------
// CSR bus
//------------------------------------------------------------------
wire [13:0]	csr_a;
wire		csr_we;
wire [31:0]	csr_dw;
wire [31:0]	csr_dr_uart,
		csr_dr_sysctl,
		csr_dr_hpdmc,
		csr_dr_ethernet,

//------------------------------------------------------------------
// FML master wires
//------------------------------------------------------------------
wire [`SDRAM_DEPTH-1:0]	fml_brg_adr;

wire			fml_brg_stb;

wire			fml_brg_we;

wire			fml_brg_ack;

wire [7:0]		fml_brg_sel;

wire [63:0]		fml_brg_dw;

wire [63:0]		fml_brg_dr;

//------------------------------------------------------------------
// FML slave wires, to memory controller
//------------------------------------------------------------------
wire [`SDRAM_DEPTH-1:0] fml_adr;
wire fml_stb;
wire fml_we;
wire fml_eack;
wire [7:0] fml_sel;
wire [63:0] fml_dw;
wire [63:0] fml_dr;

//---------------------------------------------------------------------------
// FML arbiter
//---------------------------------------------------------------------------
fmlarb #(
	.fml_depth(`SDRAM_DEPTH)
) fmlarb (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	/* WISHBONE bridge */
	.m1_adr(fml_brg_adr),
	.m1_stb(fml_brg_stb),
	.m1_we(fml_brg_we),
	.m1_ack(fml_brg_ack),
	.m1_sel(fml_brg_sel),
	.m1_di(fml_brg_dw),
	.m1_do(fml_brg_dr),

	.s_adr(fml_adr),
	.s_stb(fml_stb),
	.s_we(fml_we),
	.s_eack(fml_eack),
	.s_sel(fml_sel),
	.s_di(fml_dr),
	.s_do(fml_dw)
);

//---------------------------------------------------------------------------
// WISHBONE to CSR bridge
//---------------------------------------------------------------------------
csrbrg csrbrg(
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.wb_adr_i(csrbrg_adr),
	.wb_dat_i(csrbrg_dat_w),
	.wb_dat_o(csrbrg_dat_r),
	.wb_cyc_i(csrbrg_cyc),
	.wb_stb_i(csrbrg_stb),
	.wb_we_i(csrbrg_we),
	.wb_ack_o(csrbrg_ack),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_do(csr_dw),
	/* combine all slave->master data lines with an OR */
	.csr_di(
		 csr_dr_uart
		|csr_dr_sysctl
		|csr_dr_hpdmc
		|csr_dr_ethernet
	)
);

//---------------------------------------------------------------------------
// WISHBONE to FML bridge
//---------------------------------------------------------------------------
wire dcb_stb;
wire [`SDRAM_DEPTH-1:0] dcb_adr;
wire [63:0] dcb_dat;
wire dcb_hit;

fmlbrg #(
	.fml_depth(`SDRAM_DEPTH)
) fmlbrg (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.wb_adr_i(brg_adr),
	.wb_cti_i(brg_cti),
	.wb_dat_o(brg_dat_r),
	.wb_dat_i(brg_dat_w),
	.wb_sel_i(brg_sel),
	.wb_stb_i(brg_stb),
	.wb_cyc_i(brg_cyc),
	.wb_ack_o(brg_ack),
	.wb_we_i(brg_we),

	.fml_adr(fml_brg_adr),
	.fml_stb(fml_brg_stb),
	.fml_we(fml_brg_we),
	.fml_ack(fml_brg_ack),
	.fml_sel(fml_brg_sel),
	.fml_di(fml_brg_dr),
	.fml_do(fml_brg_dw),

	.dcb_stb(dcb_stb),
	.dcb_adr(dcb_adr),
	.dcb_dat(dcb_dat),
	.dcb_hit(dcb_hit)
);

//---------------------------------------------------------------------------
// Interrupts
//---------------------------------------------------------------------------
wire uart_irq;
wire gpio_irq;
wire timer0_irq;
wire timer1_irq;
wire ac97crrequest_irq;
wire ac97crreply_irq;
wire ac97dmar_irq;
wire ac97dmaw_irq;
wire pfpu_irq;
wire tmu_irq;
wire ethernetrx_irq;
wire ethernettx_irq;
wire videoin_irq;
wire midi_irq;
wire ir_irq;
wire usb_irq;

wire [31:0] cpu_interrupt;
assign cpu_interrupt = {16'd0,
	usb_irq,
	ir_irq,
	midi_irq,
	videoin_irq,
	ethernettx_irq,
	ethernetrx_irq,
	tmu_irq,
	pfpu_irq,
	ac97dmaw_irq,
	ac97dmar_irq,
	ac97crreply_irq,
	ac97crrequest_irq,
	timer1_irq,
	timer0_irq,
	gpio_irq,
	uart_irq
};

//---------------------------------------------------------------------------
// LM32 CPU
//---------------------------------------------------------------------------
wire bus_errors_en;
wire cpuibus_err;
wire cpudbus_err;
`ifdef CFG_BUS_ERRORS_ENABLED
// Catch NULL pointers and similar errors
// NOTE: ERR is asserted at the same time as ACK, which violates
// Wishbone rule 3.45. But LM32 doesn't care.
reg locked_addr_i;
reg locked_addr_d;
always @(posedge sys_clk) begin
	locked_addr_i <= cpuibus_adr[31:18] == 14'd0;
	locked_addr_d <= cpudbus_adr[31:18] == 14'd0;
end
assign cpuibus_err = bus_errors_en & locked_addr_i & cpuibus_ack;
assign cpudbus_err = bus_errors_en & locked_addr_d & cpudbus_ack;
`else
assign cpuibus_err = 1'b0;
assign cpudbus_err = 1'b0;
`endif

wire ext_break;
lm32_top cpu(
	.clk_i(sys_clk),
	.rst_i(sys_rst),
	.interrupt(cpu_interrupt),

	.I_ADR_O(cpuibus_adr),
	.I_DAT_I(cpuibus_dat_r),
`ifdef CFG_HW_DEBUG_ENABLED
	.I_DAT_O(cpuibus_dat_w),
	.I_SEL_O(cpuibus_sel),
`else
	.I_DAT_O(),
	.I_SEL_O(),
`endif
	.I_CYC_O(cpuibus_cyc),
	.I_STB_O(cpuibus_stb),
	.I_ACK_I(cpuibus_ack),
`ifdef CFG_HW_DEBUG_ENABLED
	.I_WE_O(cpuibus_we),
`else
	.I_WE_O(),
`endif
	.I_CTI_O(cpuibus_cti),
	.I_LOCK_O(),
	.I_BTE_O(),
	.I_ERR_I(cpuibus_err),
	.I_RTY_I(1'b0),
`ifdef CFG_EXTERNAL_BREAK_ENABLED
	.ext_break(ext_break),
`endif

	.D_ADR_O(cpudbus_adr),
	.D_DAT_I(cpudbus_dat_r),
	.D_DAT_O(cpudbus_dat_w),
	.D_SEL_O(cpudbus_sel),
	.D_CYC_O(cpudbus_cyc),
	.D_STB_O(cpudbus_stb),
	.D_ACK_I(cpudbus_ack),
	.D_WE_O (cpudbus_we),
	.D_CTI_O(cpudbus_cti),
	.D_LOCK_O(),
	.D_BTE_O(),
	.D_ERR_I(cpudbus_err),
	.D_RTY_I(1'b0)
);

//---------------------------------------------------------------------------
// Boot ROM
//---------------------------------------------------------------------------
norflash16 #(
	.adr_width(24)
) norflash (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.wb_adr_i(norflash_adr),
	.wb_dat_o(norflash_dat_r),
	.wb_dat_i(norflash_dat_w),
	.wb_sel_i(norflash_sel),
	.wb_stb_i(norflash_stb),
	.wb_cyc_i(norflash_cyc),
	.wb_ack_o(norflash_ack),
	.wb_we_i(norflash_we),

	.flash_adr(flash_adr),
	.flash_d(flash_d),
	.flash_oe_n(flash_oe_n),
	.flash_we_n(flash_we_n)
);

assign flash_ce_n = 1'b0;

//---------------------------------------------------------------------------
// UART
//---------------------------------------------------------------------------
uart #(
	.csr_addr(4'h0),
	.clk_freq(`CLOCK_FREQUENCY),
	.baud(`BAUD_RATE),
	.break_en_default(1'b1)
) uart (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_di(csr_dw),
	.csr_do(csr_dr_uart),

	.irq(uart_irq),

	.uart_rx(uart_rx),
	.uart_tx(uart_tx),

`ifdef CFG_EXTERNAL_BREAK_ENABLED
	.break(ext_break)
`else
	.break()
`endif
);

//---------------------------------------------------------------------------
// System Controller
//---------------------------------------------------------------------------
wire [13:0] gpio_outputs;
wire [31:0] capabilities;

sysctl #(
	.csr_addr(4'h1),
	.ninputs(7),
	.noutputs(2),
	.clk_freq(`CLOCK_FREQUENCY),
	.systemid(32'h12004D31) /* 1.2.0 final (0) on M1 */
) sysctl (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.gpio_irq(gpio_irq),
	.timer0_irq(timer0_irq),
	.timer1_irq(timer1_irq),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_di(csr_dw),
	.csr_do(csr_dr_sysctl),

	.gpio_inputs({pcb_revision, btn3, btn2, btn1}),
	.gpio_outputs({led2, led1}),

	.debug_write_lock(debug_write_lock),
	.bus_errors_en(bus_errors_en),

	.capabilities(capabilities),
	.hard_reset(hard_reset)
);

gen_capabilities gen_capabilities(
	.capabilities(capabilities)
);

//---------------------------------------------------------------------------
// DDR SDRAM
//---------------------------------------------------------------------------
ddram #(
	.csr_addr(4'h2)
) ddram (
	.sys_clk(sys_clk),
	.sys_clk_n(sys_clk_n),
	.sys_rst(sys_rst),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_di(csr_dw),
	.csr_do(csr_dr_hpdmc),

	.fml_adr(fml_adr),
	.fml_stb(fml_stb),
	.fml_we(fml_we),
	.fml_eack(fml_eack),
	.fml_sel(fml_sel),
	.fml_di(fml_dw),
	.fml_do(fml_dr),

	.sdram_clk_p(sdram_clk_p),
	.sdram_clk_n(sdram_clk_n),
	.sdram_cke(sdram_cke),
	.sdram_cs_n(sdram_cs_n),
	.sdram_we_n(sdram_we_n),
	.sdram_cas_n(sdram_cas_n),
	.sdram_ras_n(sdram_ras_n),
	.sdram_dm(sdram_dm),
	.sdram_adr(sdram_adr),
	.sdram_ba(sdram_ba),
	.sdram_dq(sdram_dq),
	.sdram_dqs(sdram_dqs)
);

`ifdef ENABLE_MEMTEST
memtest #(
	.csr_addr(4'h7),
	.fml_depth(`SDRAM_DEPTH)
) memtest (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_di(csr_dw),
	.csr_do(csr_dr_tmu)
);
assign tmu_irq = 1'b0;

assign tmumbus_adr = 32'hx;
assign tmumbus_cti = 3'bxxx;
assign tmumbus_cyc = 1'b0;
assign tmumbus_stb = 1'b0;

`else
assign csr_dr_tmu = 32'd0;

assign tmu_irq = 1'b0;

assign tmumbus_adr = 32'hx;
assign tmumbus_cti = 3'bxxx;
assign tmumbus_cyc = 1'b0;
assign tmumbus_stb = 1'b0;

`endif
`endif

//---------------------------------------------------------------------------
// Ethernet
//---------------------------------------------------------------------------
wire phy_tx_clk_b;
BUFG b_phy_tx_clk(
	.I(phy_tx_clk),
	.O(phy_tx_clk_b)
);
wire phy_rx_clk_b;
BUFG b_phy_rx_clk(
	.I(phy_rx_clk),
	.O(phy_rx_clk_b)
);
`ifdef ENABLE_ETHERNET
minimac2 #(
	.csr_addr(4'h8)
) ethernet (
	.sys_clk(sys_clk),
	.sys_rst(sys_rst),

	.csr_a(csr_a),
	.csr_we(csr_we),
	.csr_di(csr_dw),
	.csr_do(csr_dr_ethernet),

	.irq_rx(ethernetrx_irq),
	.irq_tx(ethernettx_irq),
	
	.wb_adr_i(eth_adr),
	.wb_dat_o(eth_dat_r),
	.wb_dat_i(eth_dat_w),
	.wb_sel_i(eth_sel),
	.wb_stb_i(eth_stb),
	.wb_cyc_i(eth_cyc),
	.wb_ack_o(eth_ack),
	.wb_we_i(eth_we),

	.phy_tx_clk(phy_tx_clk_b),
	.phy_tx_data(phy_tx_data),
	.phy_tx_en(phy_tx_en),
	.phy_tx_er(phy_tx_er),
	.phy_rx_clk(phy_rx_clk_b),
	.phy_rx_data(phy_rx_data),
	.phy_dv(phy_dv),
	.phy_rx_er(phy_rx_er),
	.phy_col(phy_col),
	.phy_crs(phy_crs),
	.phy_mii_clk(phy_mii_clk),
	.phy_mii_data(phy_mii_data),
	.phy_rst_n(phy_rst_n)
);
`else
assign csr_dr_ethernet = 32'd0;
assign eth_dat_r = 32'bx;
assign eth_ack = 1'b0;
assign ethernetrx_irq = 1'b0;
assign ethernettx_irq = 1'b0;
assign phy_tx_data = 4'b0;
assign phy_tx_en = 1'b0;
assign phy_tx_er = 1'b0;
assign phy_mii_clk = 1'b0;
assign phy_mii_data = 1'bz;
assign phy_rst_n = 1'b0;
`endif

always @(posedge clk50) phy_clk <= ~phy_clk;

endmodule
