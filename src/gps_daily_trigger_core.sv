//`default_nettype none

// Minimal daily UTC trigger generator for a pre-configured GPS receiver.
//
// Configuration is write-only and consists only of hour, minute and second.
// All three fields must be written for the schedule to become armed. Starting
// a new configuration write sequence automatically disarms the old schedule.
//
// A checksum-valid RMC timestamp T is associated with the next PPS edge. With
// ASSOCIATION_ADD_ONE=1, that PPS is labelled T+1 second.
module gps_daily_trigger_core #(
    parameter integer CLK_FREQ_HZ        = 100_000_000,
    parameter integer UART_BAUD          = 9_600,
    parameter integer PULSE_WIDTH_MS     = 50,
    parameter integer ACCEPT_GN          = 0,
    parameter integer CHECK_CHECKSUM     = 1,
    parameter integer REQUIRE_FIX_VALID  = 1,
    parameter integer ASSOCIATION_ADD_ONE = 1
) (
    input  logic       clk_i,
    input  logic       rst_ni,
    input  logic       gps_uart_rx_i,
    input  logic       gps_pps_i,

    // Minimal write-only configuration port.
    // cfg_field_i: 0=hour, 1=minute, 2=second.
    input  logic       cfg_wr_i,
    input  logic [1:0] cfg_field_i,
    input  logic [7:0] cfg_data_i,

    // Synchronous active-high clear for sticky error outputs.
    input  logic       clear_errors_i,

    output logic       trigger_o,
    output logic       gps_data_good_o,
    output logic       gps_data_bad_o,
    output logic       gps_fix_invalid_o,
    output logic       config_error_o,
    output logic       pps_tick_o,
    output logic       pps_seen_o,
    output logic       time_locked_o,
    output logic       armed_o
);
    localparam logic [1:0] CFG_HOUR   = 2'd0;
    localparam logic [1:0] CFG_MINUTE = 2'd1;
    localparam logic [1:0] CFG_SECOND = 2'd2;

    localparam integer PULSE_CYCLES_RAW =
        (CLK_FREQ_HZ / 1000) * PULSE_WIDTH_MS;
    localparam integer PULSE_CYCLES =
        (PULSE_CYCLES_RAW < 1) ? 1 : PULSE_CYCLES_RAW;
    localparam integer PULSE_COUNT_W =
        (PULSE_CYCLES <= 1) ? 1 : $clog2(PULSE_CYCLES);
    localparam logic [PULSE_COUNT_W-1:0] PULSE_RELOAD = PULSE_CYCLES - 1;

    logic [7:0] uart_byte;
    logic uart_byte_valid;
    logic uart_frame_error;

    logic parser_time_valid;
    logic [16:0] parser_time_hms;
    logic parser_bad;
    logic parser_fix_invalid;

    logic pps_meta_q;
    logic pps_sync_q;
    logic pps_sync_d_q;
    logic pps_rise;

    logic pending_valid_q;
    logic pending_match_q;

    logic [4:0] target_hour_q;
    logic [5:0] target_minute_q;
    logic [5:0] target_second_q;
    logic [2:0] cfg_written_q;
    logic armed_q;

    logic pulse_active_q;
    logic [PULSE_COUNT_W-1:0] pulse_count_q;

    logic gps_ever_good_q;
    logic [1:0] gps_good_age_q;
    logic gps_bad_sticky_q;
    logic fix_invalid_sticky_q;
    logic config_error_sticky_q;
    logic pps_seen_q;
    logic time_locked_q;

    logic selected_valid;
    logic selected_match;
    logic [16:0] parser_associated_hms;
    logic parser_target_match;

    logic [2:0] cfg_base_mask;
    logic [2:0] cfg_updated_mask;
    logic cfg_value_valid;

    function automatic logic [16:0] increment_hms(input logic [16:0] hms);
        logic [4:0] hour;
        logic [5:0] minute;
        logic [5:0] second;
        begin
            hour   = hms[16:12];
            minute = hms[11:6];
            second = hms[5:0];

            if (second == 6'd59) begin
                second = 6'd0;
                if (minute == 6'd59) begin
                    minute = 6'd0;
                    if (hour == 5'd23)
                        hour = 5'd0;
                    else
                        hour = hour + 1'b1;
                end else begin
                    minute = minute + 1'b1;
                end
            end else begin
                second = second + 1'b1;
            end

            increment_hms = {hour, minute, second};
        end
    endfunction

    uart_rx_compact #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUD_RATE  (UART_BAUD)
    ) u_uart_rx (
        .clk_i         (clk_i),
        .rst_ni        (rst_ni),
        .serial_i      (gps_uart_rx_i),
        .data_o        (uart_byte),
        .data_valid_o  (uart_byte_valid),
        .frame_error_o (uart_frame_error)
    );

    nmea_rmc_time_parser #(
        .ACCEPT_GN        (ACCEPT_GN),
        .CHECK_CHECKSUM   (CHECK_CHECKSUM),
        .REQUIRE_FIX_VALID(REQUIRE_FIX_VALID)
    ) u_parser (
        .clk_i                  (clk_i),
        .rst_ni                 (rst_ni),
        .byte_i                 (uart_byte),
        .byte_valid_i           (uart_byte_valid),
        .uart_frame_error_i     (uart_frame_error),
        .time_valid_pulse_o     (parser_time_valid),
        .time_hms_o             (parser_time_hms),
        .sentence_bad_pulse_o   (parser_bad),
        .fix_invalid_pulse_o    (parser_fix_invalid)
    );

    // Synchronize PPS into clk_i and detect its rising edge.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pps_meta_q   <= 1'b0;
            pps_sync_q   <= 1'b0;
            pps_sync_d_q <= 1'b0;
        end else begin
            pps_meta_q   <= gps_pps_i;
            pps_sync_q   <= pps_meta_q;
            pps_sync_d_q <= pps_sync_q;
        end
    end

    assign pps_rise = pps_sync_q & ~pps_sync_d_q;

    // Compute the schedule match when the RMC sentence is decoded. Only the
    // one-bit pending result must be retained until PPS; the full timestamp is
    // not stored in the core. A same-cycle parser result takes priority.
    assign parser_associated_hms = (ASSOCIATION_ADD_ONE != 0) ?
                                   increment_hms(parser_time_hms) : parser_time_hms;
    assign parser_target_match =
        (parser_associated_hms[16:12] == target_hour_q) &&
        (parser_associated_hms[11:6]  == target_minute_q) &&
        (parser_associated_hms[5:0]   == target_second_q);

    assign selected_valid = pending_valid_q | parser_time_valid;
    assign selected_match = parser_time_valid ? parser_target_match : pending_match_q;

    // Starting a new three-field update after an armed configuration clears
    // the previous field mask. This prevents an intermediate target from
    // becoming active while hour/minute/second are written separately.
    always_comb begin
        if (armed_q || (cfg_written_q == 3'b111))
            cfg_base_mask = 3'b000;
        else
            cfg_base_mask = cfg_written_q;

        cfg_updated_mask = cfg_base_mask;
        cfg_value_valid  = 1'b0;

        unique case (cfg_field_i)
            CFG_HOUR: begin
                cfg_value_valid    = (cfg_data_i <= 8'd23);
                cfg_updated_mask[0] = cfg_value_valid;
            end
            CFG_MINUTE: begin
                cfg_value_valid    = (cfg_data_i <= 8'd59);
                cfg_updated_mask[1] = cfg_value_valid;
            end
            CFG_SECOND: begin
                cfg_value_valid    = (cfg_data_i <= 8'd59);
                cfg_updated_mask[2] = cfg_value_valid;
            end
            default: begin
                cfg_value_valid = 1'b0;
            end
        endcase
    end

    assign trigger_o         = pulse_active_q;
    assign gps_data_good_o   = gps_ever_good_q && (gps_good_age_q < 2'd3);
    assign gps_data_bad_o    = gps_bad_sticky_q;
    assign gps_fix_invalid_o = fix_invalid_sticky_q;
    assign config_error_o    = config_error_sticky_q;
    assign pps_tick_o = pps_sync_q;
    assign pps_seen_o        = pps_seen_q;
    assign time_locked_o     = time_locked_q;
    assign armed_o           = armed_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            pending_valid_q       <= 1'b0;
            pending_match_q       <= 1'b0;
            target_hour_q         <= '0;
            target_minute_q       <= '0;
            target_second_q       <= '0;
            cfg_written_q         <= 3'b000;
            armed_q               <= 1'b0;
            pulse_active_q        <= 1'b0;
            pulse_count_q         <= '0;
            gps_ever_good_q       <= 1'b0;
            gps_good_age_q        <= 2'd3;
            gps_bad_sticky_q      <= 1'b0;
            fix_invalid_sticky_q  <= 1'b0;
            config_error_sticky_q <= 1'b0;
            pps_seen_q            <= 1'b0;
            time_locked_q         <= 1'b0;
        end else begin
            if (clear_errors_i) begin
                gps_bad_sticky_q      <= 1'b0;
                fix_invalid_sticky_q  <= 1'b0;
                config_error_sticky_q <= 1'b0;
            end

            if (parser_bad)
                gps_bad_sticky_q <= 1'b1;

            if (parser_fix_invalid)
                fix_invalid_sticky_q <= 1'b1;

            if (parser_time_valid) begin
                pending_valid_q <= 1'b1;
                pending_match_q <= parser_target_match;
                gps_ever_good_q <= 1'b1;
                gps_good_age_q  <= 2'd0;
            end

            // The only runtime-configurable values are target H/M/S.
            if (cfg_wr_i) begin
                armed_q       <= 1'b0;
                cfg_written_q <= cfg_updated_mask;

                if (cfg_value_valid) begin
                    unique case (cfg_field_i)
                        CFG_HOUR:   target_hour_q   <= cfg_data_i[4:0];
                        CFG_MINUTE: target_minute_q <= cfg_data_i[5:0];
                        CFG_SECOND: target_second_q <= cfg_data_i[5:0];
                        default: ;
                    endcase

                    if (cfg_updated_mask == 3'b111)
                        armed_q <= 1'b1;
                end else begin
                    config_error_sticky_q <= 1'b1;
                end
            end

            // Fixed-length pulse engine.
            if (pulse_active_q) begin
                if (pulse_count_q == '0) begin
                    pulse_active_q <= 1'b0;
                end else begin
                    pulse_count_q <= pulse_count_q - 1'b1;
                end
            end

            if (pps_rise) begin
                pps_seen_q <= 1'b1;

                if (!parser_time_valid && (gps_good_age_q != 2'd3))
                    gps_good_age_q <= gps_good_age_q + 1'b1;

                if (selected_valid) begin
                    time_locked_q   <= 1'b1;
                    pending_valid_q <= 1'b0;

                    if (armed_q && selected_match) begin
                        pulse_active_q <= 1'b1;
                        pulse_count_q  <= PULSE_RELOAD;
                    end
                end else begin
                    // Lock means the most recent PPS was paired with a fresh,
                    // accepted RMC timestamp. No large missing-PPS timer is kept.
                    time_locked_q <= 1'b0;
                end
            end
        end
    end
endmodule

`default_nettype wire
