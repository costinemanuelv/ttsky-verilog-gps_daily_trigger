module tt_um_gps_daily_trigger (
    input  logic [7:0] ui_in,
    output logic [7:0] uo_out,

    input  logic [7:0] uio_in,
    output logic [7:0] uio_out,
    output logic [7:0] uio_oe,

    input  logic       ena,
    input  logic       clk,
    input  logic       rst_n
);

    logic trigger;
    logic gps_good;
    logic gps_bad;
    logic fix_invalid;
    logic config_error;
    logic pps_tick;
    logic time_locked;
    logic armed;

    gps_daily_trigger_core #(
        .CLK_FREQ_HZ     (50_000_000),
        .UART_BAUD       (9_600),
        .PULSE_WIDTH_MS  (50),
        .CHECK_CHECKSUM  (1),
        .REQUIRE_FIX_VALID(1)
    ) u_core (
        .clk_i             (clk),
        .rst_ni            (rst_n),

        .gps_uart_rx_i     (ui_in[0]),
        .gps_pps_i         (ui_in[1]),

        .clear_errors_i    (ui_in[2]),

        .cfg_wr_i          (ui_in[3]),
        .cfg_field_i       (ui_in[5:4]),
        .cfg_data_i        (uio_in),

        .trigger_o         (trigger),
        .gps_data_good_o   (gps_good),
        .gps_data_bad_o    (gps_bad),
        .gps_fix_invalid_o (fix_invalid),
        .config_error_o    (config_error),
        .pps_tick_o        (pps_tick),
        .time_locked_o     (time_locked),
        .armed_o           (armed)
    );

    always_comb begin
        uo_out[0] = trigger;
        uo_out[1] = gps_good;
        uo_out[2] = gps_bad;
        uo_out[3] = fix_invalid;
        uo_out[4] = config_error;
        uo_out[5] = pps_tick;
        uo_out[6] = time_locked;
        uo_out[7] = armed;
    end

    // All bidirectional pins are configuration inputs.
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;

    // ena and unused inputs are intentionally unused.
    logic unused;
    always_comb begin
        unused = &{ena, ui_in[7:6], 1'b0};
    end

endmodule

