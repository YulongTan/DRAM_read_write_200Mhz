`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/08/29 19:26:05
// Design Name: 
// Module Name: tb_write_read
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


module tb_write_read;

    // 信号声明
    reg clk;
    reg rst_n;
    reg IO_EN;
    reg [1:0] IO_MODEL;
    reg [16:1] DRAM16_data;
    wire RD_DONE;
    wire WT_DONE;
    wire [2:0] PC_data;
    wire ADD_IN;
    wire ADD_VALID_IN;
    wire [1:0] PC_D_IN;
    wire [16:1] D_IN;
    wire DATA_VALID_IN;
    wire clk_out;
    wire WRI_EN;
    wire [16:1] R_AD;
    wire [1:0] PC_R_AD;
    wire [16:1] LIM_IN;
    wire [1:0] LIM_SEL;
    wire DE_ADD3;
    wire RD_EN;
    wire VSAEN;
    wire REF_WWL;
    initial begin 
        clk = 0;
        forever #5 clk = ~clk;
    end
    initial begin
        // rst_n = 0;
        // IO_EN = 0;
        // IO_MODEL = 2'b00;
        // DRAM16_data = 16'hA5A5;
        // #600;
        // #10;
        // rst_n = 1;
        // #10;
        // IO_EN = 1;
        // IO_MODEL = 2'b01;
        // #10;
        // IO_EN = 0;
        // // #10;
        // // IO_EN = 1;
        // // IO_MODEL = 2'b10;
        // // #10;
        // // IO_EN = 0;
        // #10;
        // wait(WT_DONE);
        
        rst_n = 0;
        IO_EN = 0;
        IO_MODEL = 2'b00;
        DRAM16_data = 16'hA5A5;
        #600;
        #10;
        rst_n = 1;
        #10;
        IO_EN = 1;
        IO_MODEL = 2'b10;
        // #10;
        // IO_EN = 0;
        // #10;
        // IO_EN = 1;
        // IO_MODEL = 2'b10;
        #10;
        IO_EN = 0;
        #10;
        wait(RD_DONE);
        $finish;
    end

    // DUT实例化
    // test_write_read utest_write_read (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .IO_EN(IO_EN),
    //     .IO_MODEL(IO_MODEL),
    //     .DRAM16_data(DRAM16_data),
    //     .RD_DONE(RD_DONE),
    //     .WT_DONE(WT_DONE),
    //     .PC_data(PC_data),
    //     .ADD_IN(ADD_IN),
    //     .ADD_VALID_IN(ADD_VALID_IN),
    //     .PC_D_IN(PC_D_IN),
    //     .D_IN(D_IN),
    //     .DATA_VALID_IN(DATA_VALID_IN),
    //     .clk_out(clk_out),
    //     .WRI_EN(WRI_EN),
    //     .R_AD(R_AD),
    //     .PC_R_AD(PC_R_AD),
    //     .LIM_IN(LIM_IN),
    //     .LIM_SEL(LIM_SEL),
    //     .DE_ADD3(DE_ADD3),
    //     .RD_EN(RD_EN),
    //     .VSAEN(VSAEN),
    //     .REF_WWL(REF_WWL)
    // );
    //
    test_write_read tb_test_write_read(
        .clk_n       (clk_n       ),
        .clk_p       (clk_p       ),
        .rst_n       (rst_n       ),
        .IO_EN_button(IO_EN_button),
        .DRAM16_data (DRAM16_data ), // DRAM芯片输入数据
        .RD_DONE_LED (RD_DONE_LED ), // DRAM_DATA_OUT done信号
        .WT_DONE_LED  (WT_DONE_LED  ), // DRAM写入完成done信号
        .PC_data      (PC_data      ), /// PC并转串控制信号 PC[0]=clk PC[1]=SR/LD# PC[2]=CLK_INV
        .ADD_IN       (ADD_IN       ),            // ADD_IN // WWL_ADD 输入 自带CP 1 to 6
        .ADD_VALID_IN (ADD_VALID_IN ),      // A_VALID// WWL_ADD_VALID 输入地址使能
        .PC_D_IN      (PC_D_IN      ),      /// D_IN 的串转并控制信号 PC_D_IN[1]为rst_n  PC_D_IN[0]为移位时钟
        .D_IN         (D_IN         ),        /// D_IN[1:16] // 16块芯片的DATA_I
        .DATA_VALID_IN (DATA_VALID_IN ),     // D_VALIDv// WBL 输入数据使能
        .clk_out      (clk_out      ),           // 相当于带使能的100MHz时钟
        .WRI_EN       (WRI_EN       ),            // WRI_EN 写使能
        .R_AD        (R_AD        ),        ///R_AD 读/算地址 串转并后高两位是DE_ADD0 1
        .PC_R_AD      (PC_R_AD      ),      ///R_AD 的串转并控制信号
        .LIM_IN       (LIM_IN       ),     /// LIM输入 16块芯片的算输入数据
        .LIM_SEL      (LIM_SEL      ),    /// LIM_SEL 存算模式选择
        .DE_ADD3      (DE_ADD3      ),           /// DE_ADD3
        .RD_EN        (RD_EN        ),         // 读使能 RWL_EN
        .VSAEN       (VSAEN       ),
        .REF_WWL     (REF_WWL     ),
        .uart_txd    (uart_txd    ),        // 串口发送脚
        .SW2         (SW2         ),
        .SW3         (SW3         ),
        .SW4         (SW4         ),
        .SW5         (SW5         ),
        .SW6         (SW6         ),
        .SW7         (SW7         ),
        .SW8         (SW8         ),
        .SW9         (SW9         ),
        .SW10       (SW10       ),
        .SW11       (SW11       ),
        .SW12       (SW12       )
    );  
    // 时钟和复位等激励可在此添加

endmodule
