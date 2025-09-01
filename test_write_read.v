`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  
// Engineer: 
// 
// Create Date: 2025/08/29 16:11:23
// Design Name: 
// Module Name: test_write_read
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module test_write_read(
    input wire clk,  // 100MHz
    input wire rst_n, 
    input wire IO_EN_button,
    // input wire [1:0] IO_MODEL, // IO模型选择
    input wire [16:1] DRAM16_data, // DRAM芯片输入数据
    output wire RD_DONE_LED, // DRAM_DATA_OUT done信号
    output wire WT_DONE_LED, // DRAM写入完成done信号
    // DRAM_IO
    // IO数据DRAM➡FPGA
    // input wire [8:1] DRAM_data, // 单芯片
    output wire [2:0]PC_data,      /// PC并转串控制信号 PC[0]=clk PC[1]=SR/LD# PC[2]=CLK_INV
    // IO控制FPGA➡DRAM
    output wire ADD_IN,            // ADD_IN // WWL_ADD 输入 自带CP 1 to 6
    output wire ADD_VALID_IN,      // A_VALID// WWL_ADD_VALID 输入地址使能
    output wire [1:0]PC_D_IN,      /// D_IN 的串转并控制信号 PC_D_IN[1]为rst_n  PC_D_IN[0]为移位时钟
    output wire [16:1]D_IN,        /// D_IN[1:16] // 16块芯片的DATA_I
    output wire DATA_VALID_IN,     // D_VALIDv// WBL 输入数据使能
    output wire clk_out,           // 相当于带使能的100MHz时钟
    output wire WRI_EN,            // WRI_EN 写使能
    output wire [16:1]R_AD,        ///R_AD 读/算地址 串转并后高两位是DE_ADD0 1
    output wire [1:0]PC_R_AD,      ///R_AD 的串转并控制信号
    output wire [16:1]LIM_IN,     /// LIM输入 16块芯片的算输入数据
    output wire [1:0] LIM_SEL,    /// LIM_SEL 存算模式选择
    output wire DE_ADD3,           /// DE_ADD3
    output wire RD_EN,         // 读使能 RWL_EN
    output wire VSAEN,
    output wire REF_WWL,
    output wire uart_txd        // 串口发送脚
);
    // generate clock
    wire clk_400m;
    wire clk_100m;
    wire clk_200m;
    wire clk_vsa;
    wire clk_locked;
    wire rst_n_locked = rst_n & clk_locked;
    clk_wiz_400m u_clk_wiz_400m(
        .clk_400m(clk_400m),
        .clk_100m(clk_100m),
        .clk_200m(clk_200m),
        .clk_vsa(clk_vsa),
        .locked(clk_locked),
        .clk(clk)
    );
    // 信号定义

    // 按键消抖及脉冲生成
    reg btn_sync0, btn_sync1;
    reg [19:0] debounce_cnt;
    reg btn_state;
    reg btn_state_d;
    wire IO_EN;

    // reg
    reg [1:0] IO_MODEL;
    reg [1:0] CIM_model;
    reg [16:1] DATA_IN;
    reg [63:0] WBL_DATA_IN1;
    reg [63:0] WBL_DATA_IN2;
    reg [63:0] WBL_DATA_IN3;
    reg [63:0] WBL_DATA_IN4;
    reg [63:0] WBL_DATA_IN5;
    reg [63:0] WBL_DATA_IN6;
    reg [63:0] WBL_DATA_IN7;
    reg [63:0] WBL_DATA_IN8;
    reg [63:0] WBL_DATA_IN9;
    reg [63:0] WBL_DATA_IN10;
    reg [63:0] WBL_DATA_IN11;
    reg [63:0] WBL_DATA_IN12;
    reg [63:0] WBL_DATA_IN13;
    reg [63:0] WBL_DATA_IN14;
    reg [63:0] WBL_DATA_IN15;
    reg [63:0] WBL_DATA_IN16;
    reg [5:0] WWL_ADD;
    // 读出
    reg [5:0] RWL_DEC_ADD1;
    reg [5:0] RWL_DEC_ADD2;
    reg [5:0] RWL_DEC_ADD3;
    reg [5:0] RWL_DEC_ADD4;
    reg [5:0] RWL_DEC_ADD5;
    reg [5:0] RWL_DEC_ADD6;
    reg [5:0] RWL_DEC_ADD7;
    reg [5:0] RWL_DEC_ADD8;
    reg [5:0] RWL_DEC_ADD9;
    reg [5:0] RWL_DEC_ADD10;
    reg [5:0] RWL_DEC_ADD11;
    reg [5:0] RWL_DEC_ADD12;
    reg [5:0] RWL_DEC_ADD13;
    reg [5:0] RWL_DEC_ADD14;
    reg [5:0] RWL_DEC_ADD15;
    reg [5:0] RWL_DEC_ADD16;
    reg [1:0] DEMUX_ADD1;
    reg [1:0] DEMUX_ADD2;
    reg [1:0] DEMUX_ADD3;
    reg [1:0] DEMUX_ADD4;
    reg [1:0] DEMUX_ADD5;
    reg [1:0] DEMUX_ADD6;
    reg [1:0] DEMUX_ADD7;
    reg [1:0] DEMUX_ADD8;
    reg [1:0] DEMUX_ADD9;
    reg [1:0] DEMUX_ADD10;
    reg [1:0] DEMUX_ADD11;
    reg [1:0] DEMUX_ADD12;
    reg [1:0] DEMUX_ADD13;
    reg [1:0] DEMUX_ADD14;
    reg [1:0] DEMUX_ADD15;
    reg [1:0] DEMUX_ADD16;
    reg DEMUX_ADD_3;
    // 按键消抖逻辑
    always @(posedge clk_100m or negedge rst_n_locked) begin
        if (!rst_n_locked) begin
            btn_sync0    <= 1'b0;
            btn_sync1    <= 1'b0;
            debounce_cnt <= 20'd0;
            btn_state    <= 1'b0;
            btn_state_d  <= 1'b0;
        end else begin
            // 同步到时钟域
            btn_sync0 <= IO_EN_button;
            btn_sync1 <= btn_sync0;

            // 消抖计数
            if (btn_sync1 != btn_state) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt == 20'hFFFFF) begin
                    btn_state    <= btn_sync1;
                    debounce_cnt <= 20'd0;
                end
            end else begin
                debounce_cnt <= 20'd0;
            end

            // 产生脉冲
            btn_state_d <= btn_state;
        end
    end

    assign IO_EN = btn_state & ~btn_state_d;

    // 输出
    wire [7:0] DRAM_DATA_OUT1 ;
    wire [7:0] DRAM_DATA_OUT2 ;
    wire [7:0] DRAM_DATA_OUT3 ;
    wire [7:0] DRAM_DATA_OUT4 ;
    wire [7:0] DRAM_DATA_OUT5 ;
    wire [7:0] DRAM_DATA_OUT6 ;
    wire [7:0] DRAM_DATA_OUT7 ;
    wire [7:0] DRAM_DATA_OUT8 ;
    wire [7:0] DRAM_DATA_OUT9 ;
    wire [7:0] DRAM_DATA_OUT10;
    wire [7:0] DRAM_DATA_OUT11;
    wire [7:0] DRAM_DATA_OUT12;
    wire [7:0] DRAM_DATA_OUT13;
    wire [7:0] DRAM_DATA_OUT14;
    wire [7:0] DRAM_DATA_OUT15;
    wire [7:0] DRAM_DATA_OUT16;
    wire RD_DONE;
    wire WT_DONE;
    // LED指示灯
    assign RD_DONE_LED = RD_DONE;
    assign WT_DONE_LED = WT_DONE;
    

    always @(posedge clk_100m or negedge rst_n) begin
        if (!rst_n) begin
            // 初始化所有寄存器
            // 读写控制
            IO_MODEL <= 2'b00;
            //
            CIM_model <= 2'b10;
            DATA_IN <= 16'hffff;
            WBL_DATA_IN1 <=  64'b0;
            WBL_DATA_IN2 <=  64'b0;
            WBL_DATA_IN3 <=  64'b0;
            WBL_DATA_IN4 <=  64'b0;
            WBL_DATA_IN5 <=  64'b0;
            WBL_DATA_IN6 <=  64'b0;
            WBL_DATA_IN7 <=  64'b0;
            WBL_DATA_IN8 <=  64'b0;
            WBL_DATA_IN9 <=  64'b0;
            WBL_DATA_IN10 <= 64'b0;
            WBL_DATA_IN11 <= 64'b0;
            WBL_DATA_IN12 <= 64'b0;
            WBL_DATA_IN13 <= 64'b0;
            WBL_DATA_IN14 <= 64'b0;
            WBL_DATA_IN15 <= 64'b0;
            WBL_DATA_IN16 <= 64'b0;
            WWL_ADD <= 6'b0;
            RWL_DEC_ADD1 <= 6'b0;
            RWL_DEC_ADD2 <= 6'b0;
            RWL_DEC_ADD3 <= 6'b0;
            RWL_DEC_ADD4 <= 6'b0;
            RWL_DEC_ADD5 <= 6'b0;
            RWL_DEC_ADD6 <= 6'b0;
            RWL_DEC_ADD7 <= 6'b0;
            RWL_DEC_ADD8 <= 6'b0;
            RWL_DEC_ADD9 <= 6'b0;
            RWL_DEC_ADD10 <= 6'b0;
            RWL_DEC_ADD11 <= 6'b0;
            RWL_DEC_ADD12 <= 6'b0;
            RWL_DEC_ADD13 <= 6'b0;
            RWL_DEC_ADD14 <= 6'b0;
            RWL_DEC_ADD15 <= 6'b0;
            RWL_DEC_ADD16 <= 6'b0;
            DEMUX_ADD1 <= 2'b0;
            DEMUX_ADD2 <= 2'b0;
            DEMUX_ADD3 <= 2'b0;
            DEMUX_ADD4 <= 2'b0;
            DEMUX_ADD5 <= 2'b0;
            DEMUX_ADD6 <= 2'b0;
            DEMUX_ADD7 <= 2'b0;
            DEMUX_ADD8 <= 2'b0;
            DEMUX_ADD9 <= 2'b0;
            DEMUX_ADD10 <= 2'b0;
            DEMUX_ADD11 <= 2'b0;
            DEMUX_ADD12 <= 2'b0;
            DEMUX_ADD13 <= 2'b0;
            DEMUX_ADD14 <= 2'b0;
            DEMUX_ADD15 <= 2'b0;
            DEMUX_ADD16 <= 2'b0;
            DEMUX_ADD_3 <= 1'b0;
        end
        else begin
            IO_MODEL <= 2'b01;
            //
            CIM_model <= 2'b10;
            DATA_IN <= 16'hffff;
            WBL_DATA_IN1 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN2 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN3 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN4 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN5 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN6 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN7 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN8 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN9 <=  {{8{8'b0101_0101}}};
            WBL_DATA_IN10 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN11 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN12 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN13 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN14 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN15 <= {{8{8'b0101_0101}}};
            WBL_DATA_IN16 <= {{8{8'b0101_0101}}};
            WWL_ADD <= 6'b0;
            RWL_DEC_ADD1 <= 6'b0;
            RWL_DEC_ADD2 <= 6'b0;
            RWL_DEC_ADD3 <= 6'b0;
            RWL_DEC_ADD4 <= 6'b0;
            RWL_DEC_ADD5 <= 6'b0;
            RWL_DEC_ADD6 <= 6'b0;
            RWL_DEC_ADD7 <= 6'b0;
            RWL_DEC_ADD8 <= 6'b0;
            RWL_DEC_ADD9 <= 6'b0;
            RWL_DEC_ADD10 <= 6'b0;
            RWL_DEC_ADD11 <= 6'b0;
            RWL_DEC_ADD12 <= 6'b0;
            RWL_DEC_ADD13 <= 6'b0;
            RWL_DEC_ADD14 <= 6'b0;
            RWL_DEC_ADD15 <= 6'b0;
            RWL_DEC_ADD16 <= 6'b0;
            DEMUX_ADD1 <= 2'b0;
            DEMUX_ADD2 <= 2'b0;
            DEMUX_ADD3 <= 2'b0;
            DEMUX_ADD4 <= 2'b0;
            DEMUX_ADD5 <= 2'b0;
            DEMUX_ADD6 <= 2'b0;
            DEMUX_ADD7 <= 2'b0;
            DEMUX_ADD8 <= 2'b0;
            DEMUX_ADD9 <= 2'b0;
            DEMUX_ADD10 <= 2'b0;
            DEMUX_ADD11 <= 2'b0;
            DEMUX_ADD12 <= 2'b0;
            DEMUX_ADD13 <= 2'b0;
            DEMUX_ADD14 <= 2'b0;
            DEMUX_ADD15 <= 2'b0;
            DEMUX_ADD16 <= 2'b0;
            DEMUX_ADD_3 <= 1'b0;
        end
    end

    // 实例化
    DRAM_write_read_16core u_DRAM_write_read_16core (
        .clk_100m(clk_100m),
        .clk_400m(clk_400m),
        .clk_200m(clk_200m),
        .clk_vsa(clk_vsa),
        .rst_n(rst_n_locked),
        .IO_EN(IO_EN),
        .IO_MODEL(IO_MODEL),
        .CIM_model(CIM_model),
        .DATA_IN(DATA_IN),
        .WBL_DATA_IN1(WBL_DATA_IN1),
        .WBL_DATA_IN2(WBL_DATA_IN2),
        .WBL_DATA_IN3(WBL_DATA_IN3),
        .WBL_DATA_IN4(WBL_DATA_IN4),
        .WBL_DATA_IN5(WBL_DATA_IN5),
        .WBL_DATA_IN6(WBL_DATA_IN6),
        .WBL_DATA_IN7(WBL_DATA_IN7),
        .WBL_DATA_IN8(WBL_DATA_IN8),
        .WBL_DATA_IN9(WBL_DATA_IN9),
        .WBL_DATA_IN10(WBL_DATA_IN10),
        .WBL_DATA_IN11(WBL_DATA_IN11),
        .WBL_DATA_IN12(WBL_DATA_IN12),
        .WBL_DATA_IN13(WBL_DATA_IN13),
        .WBL_DATA_IN14(WBL_DATA_IN14),
        .WBL_DATA_IN15(WBL_DATA_IN15),
        .WBL_DATA_IN16(WBL_DATA_IN16),
        .WWL_ADD(WWL_ADD),
        .RWL_DEC_ADD1(RWL_DEC_ADD1),
        .RWL_DEC_ADD2(RWL_DEC_ADD2),
        .RWL_DEC_ADD3(RWL_DEC_ADD3),
        .RWL_DEC_ADD4(RWL_DEC_ADD4),
        .RWL_DEC_ADD5(RWL_DEC_ADD5),
        .RWL_DEC_ADD6(RWL_DEC_ADD6),
        .RWL_DEC_ADD7(RWL_DEC_ADD7),
        .RWL_DEC_ADD8(RWL_DEC_ADD8),
        .RWL_DEC_ADD9(RWL_DEC_ADD9),
        .RWL_DEC_ADD10(RWL_DEC_ADD10),
        .RWL_DEC_ADD11(RWL_DEC_ADD11),
        .RWL_DEC_ADD12(RWL_DEC_ADD12),
        .RWL_DEC_ADD13(RWL_DEC_ADD13),
        .RWL_DEC_ADD14(RWL_DEC_ADD14),
        .RWL_DEC_ADD15(RWL_DEC_ADD15),
        .RWL_DEC_ADD16(RWL_DEC_ADD16),
        .DEMUX_ADD1(DEMUX_ADD1),
        .DEMUX_ADD2(DEMUX_ADD2),
        .DEMUX_ADD3(DEMUX_ADD3),
        .DEMUX_ADD4(DEMUX_ADD4),
        .DEMUX_ADD5(DEMUX_ADD5),
        .DEMUX_ADD6(DEMUX_ADD6),
        .DEMUX_ADD7(DEMUX_ADD7),
        .DEMUX_ADD8(DEMUX_ADD8),
        .DEMUX_ADD9(DEMUX_ADD9),
        .DEMUX_ADD10(DEMUX_ADD10),
        .DEMUX_ADD11(DEMUX_ADD11),
        .DEMUX_ADD12(DEMUX_ADD12),
        .DEMUX_ADD13(DEMUX_ADD13),
        .DEMUX_ADD14(DEMUX_ADD14),
        .DEMUX_ADD15(DEMUX_ADD15),
        .DEMUX_ADD16(DEMUX_ADD16),
        .DEMUX_ADD_3(DEMUX_ADD_3),
        .DRAM_DATA_OUT1(DRAM_DATA_OUT1),
        .DRAM_DATA_OUT2(DRAM_DATA_OUT2),
        .DRAM_DATA_OUT3(DRAM_DATA_OUT3),
        .DRAM_DATA_OUT4(DRAM_DATA_OUT4),
        .DRAM_DATA_OUT5(DRAM_DATA_OUT5),
        .DRAM_DATA_OUT6(DRAM_DATA_OUT6),
        .DRAM_DATA_OUT7(DRAM_DATA_OUT7),
        .DRAM_DATA_OUT8(DRAM_DATA_OUT8),
        .DRAM_DATA_OUT9(DRAM_DATA_OUT9),
        .DRAM_DATA_OUT10(DRAM_DATA_OUT10),
        .DRAM_DATA_OUT11(DRAM_DATA_OUT11),
        .DRAM_DATA_OUT12(DRAM_DATA_OUT12),
        .DRAM_DATA_OUT13(DRAM_DATA_OUT13),
        .DRAM_DATA_OUT14(DRAM_DATA_OUT14),
        .DRAM_DATA_OUT15(DRAM_DATA_OUT15),
        .DRAM_DATA_OUT16(DRAM_DATA_OUT16),
        .RD_DONE(RD_DONE),
        .WT_DONE(WT_DONE),
        .DRAM16_data(DRAM16_data),
        .PC_data(PC_data),
        .ADD_IN(ADD_IN),
        .ADD_VALID_IN(ADD_VALID_IN),
        .PC_D_IN(PC_D_IN),
        .D_IN(D_IN),
        .DATA_VALID_IN(DATA_VALID_IN),
        .clk_out(clk_out),
        .WRI_EN(WRI_EN),
        .R_AD(R_AD),
        .PC_R_AD(PC_R_AD),
        .LIM_IN(LIM_IN),
        .LIM_SEL(LIM_SEL),
        .DE_ADD3(DE_ADD3),
        .RD_EN(RD_EN),
        .VSAEN(VSAEN),
        .REF_WWL(REF_WWL)
    );

    // 串口发送逻辑
    wire uart_busy;
    reg uart_en;
    reg [7:0] uart_data;
    reg [5:0] send_idx;
    reg sending;

    uart_send u_uart (
        .clk(clk_100m),
        .uart_en(uart_en),
        .uart_din(uart_data),
        .uart_txd(uart_txd),
        .uart_busy(uart_busy)
    );

    always @(posedge clk_100m or negedge rst_n_locked) begin
        if (!rst_n_locked) begin
            uart_en   <= 1'b0;
            uart_data <= 8'b0;
            send_idx  <= 6'd0;
            sending   <= 1'b0;
        end else begin
            uart_en <= 1'b0;
            if (RD_DONE && !sending) begin
                sending  <= 1'b1;
                send_idx <= 6'd0;
            end else if (sending && !uart_busy) begin
                uart_en <= 1'b1;
                case (send_idx)
                    6'd0:  uart_data <= DRAM_DATA_OUT1;
                    6'd1:  uart_data <= 8'h0A;
                    6'd2:  uart_data <= DRAM_DATA_OUT2;
                    6'd3:  uart_data <= 8'h0A;
                    6'd4:  uart_data <= DRAM_DATA_OUT3;
                    6'd5:  uart_data <= 8'h0A;
                    6'd6:  uart_data <= DRAM_DATA_OUT4;
                    6'd7:  uart_data <= 8'h0A;
                    6'd8:  uart_data <= DRAM_DATA_OUT5;
                    6'd9:  uart_data <= 8'h0A;
                    6'd10: uart_data <= DRAM_DATA_OUT6;
                    6'd11: uart_data <= 8'h0A;
                    6'd12: uart_data <= DRAM_DATA_OUT7;
                    6'd13: uart_data <= 8'h0A;
                    6'd14: uart_data <= DRAM_DATA_OUT8;
                    6'd15: uart_data <= 8'h0A;
                    6'd16: uart_data <= DRAM_DATA_OUT9;
                    6'd17: uart_data <= 8'h0A;
                    6'd18: uart_data <= DRAM_DATA_OUT10;
                    6'd19: uart_data <= 8'h0A;
                    6'd20: uart_data <= DRAM_DATA_OUT11;
                    6'd21: uart_data <= 8'h0A;
                    6'd22: uart_data <= DRAM_DATA_OUT12;
                    6'd23: uart_data <= 8'h0A;
                    6'd24: uart_data <= DRAM_DATA_OUT13;
                    6'd25: uart_data <= 8'h0A;
                    6'd26: uart_data <= DRAM_DATA_OUT14;
                    6'd27: uart_data <= 8'h0A;
                    6'd28: uart_data <= DRAM_DATA_OUT15;
                    6'd29: uart_data <= 8'h0A;
                    6'd30: uart_data <= DRAM_DATA_OUT16;
                    6'd31: uart_data <= 8'h0A;
                    default: uart_data <= 8'h00;
                endcase
                send_idx <= send_idx + 1'b1;
                if (send_idx == 6'd31) begin
                    sending <= 1'b0;
                end
            end
        end
    end

    // 后续可添加initial块进行仿真激励

endmodule
