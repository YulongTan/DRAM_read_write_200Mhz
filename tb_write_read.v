`timescale 1ns/1ps

module tb_write_read;
    // differential clock
    reg clk_p;
    reg clk_n;
    // reset and button
    reg rst_n;
    reg IO_EN_button;
    // DRAM input data
    reg [16:1] DRAM16_data;

    // switches
    reg SW2;
    reg SW3;
    reg SW4;
    reg SW5;
    reg SW6;
    reg SW7;
    reg SW8;
    reg SW9;
    reg SW10;
    reg SW11;
    reg SW12;

    // outputs from DUT
    wire RD_DONE_LED;
    wire WT_DONE_LED;
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
    wire uart_txd;

    // clock generation 100MHz differential
    initial begin
        clk_p = 0;
        clk_n = 1;
        forever begin
            #5 clk_p = ~clk_p;
            clk_n = ~clk_n;
        end
    end

    // stimulus
    initial begin
        // default values
        rst_n = 0;
        IO_EN_button = 1'b1; // not pressed (active-low)
        DRAM16_data = 16'hA5A5;
        SW2 = 0; SW3 = 0; SW4 = 0; SW5 = 0; SW6 = 0;
        SW7 = 0; SW8 = 0; SW9 = 0; SW10 = 0; SW11 = 0; SW12 = 0;

        // release reset
        #100;
        rst_n = 1'b1;

        // simulate button press to start write
        #200;
        IO_EN_button = 1'b0; // press
        #20;
        IO_EN_button = 1'b1; // release

        // wait for write complete
        wait(WT_DONE_LED);
        #1000;
        $finish;
    end

    // device under test
    test_write_read dut (
        .clk_n       (clk_n       ),
        .clk_p       (clk_p       ),
        .rst_n       (rst_n       ),
        .IO_EN_button(IO_EN_button),
        .DRAM16_data (DRAM16_data ),
        .RD_DONE_LED (RD_DONE_LED ),
        .WT_DONE_LED (WT_DONE_LED ),
        .PC_data     (PC_data     ),
        .ADD_IN      (ADD_IN      ),
        .ADD_VALID_IN(ADD_VALID_IN),
        .PC_D_IN     (PC_D_IN     ),
        .D_IN        (D_IN        ),
        .DATA_VALID_IN(DATA_VALID_IN),
        .clk_out     (clk_out     ),
        .WRI_EN      (WRI_EN      ),
        .R_AD        (R_AD        ),
        .PC_R_AD     (PC_R_AD     ),
        .LIM_IN      (LIM_IN      ),
        .LIM_SEL     (LIM_SEL     ),
        .DE_ADD3     (DE_ADD3     ),
        .RD_EN       (RD_EN       ),
        .VSAEN       (VSAEN       ),
        .REF_WWL     (REF_WWL     ),
        .uart_txd    (uart_txd    ),
        .SW2         (SW2         ),
        .SW3         (SW3         ),
        .SW4         (SW4         ),
        .SW5         (SW5         ),
        .SW6         (SW6         ),
        .SW7         (SW7         ),
        .SW8         (SW8         ),
        .SW9         (SW9         ),
        .SW10        (SW10        ),
        .SW11        (SW11        ),
        .SW12        (SW12        )
    );

endmodule
