//`default_nettype none

// Compact streaming parser for $GPRMC (and optionally $GNRMC).
//
// Only the fields needed by the trigger generator are retained:
//   field 1: UTC time, HHMMSS[.fraction]
//   field 2: RMC navigation status, A=valid and V=void
//
// The complete sentence is still consumed so its NMEA XOR checksum can be
// validated. Other NMEA sentence types are ignored and are not errors.
module nmea_rmc_time_parser #(
    parameter integer ACCEPT_GN         = 0,
    parameter integer CHECK_CHECKSUM    = 1,
    parameter integer REQUIRE_FIX_VALID = 1
) (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic [7:0]  byte_i,
    input  logic        byte_valid_i,
    input  logic        uart_frame_error_i,

    output logic        time_valid_pulse_o,
    output logic [16:0] time_hms_o,          // {hour[4:0], minute[5:0], second[5:0]}
    output logic        sentence_bad_pulse_o,
    output logic        fix_invalid_pulse_o
);
    typedef enum logic [2:0] {
        ST_WAIT_DOLLAR,
        ST_HEADER,
        ST_DATA,
        ST_CHECKSUM_HI,
        ST_CHECKSUM_LO
    } state_t;

    state_t state_q;
    logic [2:0] header_pos_q;
    logic [7:0] checksum_q;
    logic [3:0] checksum_hi_q;

    // field_index_q saturates at 3 because fields after the RMC status are not
    // otherwise interpreted. They are still included in the checksum.
    logic [1:0] field_index_q;
    logic [3:0] field_char_index_q;
    logic [2:0] time_digit_count_q;
    logic time_format_bad_q;
    logic status_seen_q;
    logic status_valid_q;

    logic [3:0] h_tens_q;
    logic [3:0] h_ones_q;
    logic [3:0] m_tens_q;
    logic [3:0] m_ones_q;
    logic [3:0] s_tens_q;
    logic [3:0] s_ones_q;

    logic [5:0] hour_value;
    logic [5:0] minute_value;
    logic [5:0] second_value;
    logic time_range_valid;

    function automatic logic is_digit(input logic [7:0] value);
        is_digit = (value >= 8'h30) && (value <= 8'h39);
    endfunction

    function automatic logic is_hex(input logic [7:0] value);
        is_hex = ((value >= 8'h30) && (value <= 8'h39)) ||
                 ((value >= 8'h41) && (value <= 8'h46)) ||
                 ((value >= 8'h61) && (value <= 8'h66));
    endfunction

    function automatic logic [3:0] hex_value(input logic [7:0] value);
        begin
            if ((value >= 8'h30) && (value <= 8'h39))
                hex_value = value - 8'h30;
            else if ((value >= 8'h41) && (value <= 8'h46))
                hex_value = value - 8'h41 + 4'd10;
            else
                hex_value = value - 8'h61 + 4'd10;
        end
    endfunction

    // Multiply a decimal tens digit by ten using shifts/addition.
    always_comb begin
        hour_value   = ({2'b00, h_tens_q} << 3) +
                       ({2'b00, h_tens_q} << 1) + h_ones_q;
        minute_value = ({2'b00, m_tens_q} << 3) +
                       ({2'b00, m_tens_q} << 1) + m_ones_q;
        second_value = ({2'b00, s_tens_q} << 3) +
                       ({2'b00, s_tens_q} << 1) + s_ones_q;

        time_range_valid = (h_tens_q <= 4'd2) &&
                           ((h_tens_q != 4'd2) || (h_ones_q <= 4'd3)) &&
                           (m_tens_q <= 4'd5) &&
                           (s_tens_q <= 4'd5);
    end

    assign time_hms_o = {hour_value[4:0], minute_value[5:0], second_value[5:0]};

    always_ff @(posedge clk_i or negedge rst_ni) begin : parser_fsm
        logic header_mismatch;
        logic [3:0] checksum_low;
        logic checksum_ok;
        logic sentence_structure_ok;

        if (!rst_ni) begin
            state_q                <= ST_WAIT_DOLLAR;
            header_pos_q           <= '0;
            checksum_q             <= '0;
            checksum_hi_q          <= '0;
            field_index_q          <= '0;
            field_char_index_q     <= '0;
            time_digit_count_q     <= '0;
            time_format_bad_q      <= 1'b0;
            status_seen_q          <= 1'b0;
            status_valid_q         <= 1'b0;
            h_tens_q               <= '0;
            h_ones_q               <= '0;
            m_tens_q               <= '0;
            m_ones_q               <= '0;
            s_tens_q               <= '0;
            s_ones_q               <= '0;
            time_valid_pulse_o     <= 1'b0;
            sentence_bad_pulse_o   <= 1'b0;
            fix_invalid_pulse_o    <= 1'b0;
        end else begin
            time_valid_pulse_o   <= 1'b0;
            sentence_bad_pulse_o <= 1'b0;
            fix_invalid_pulse_o  <= 1'b0;

            if (uart_frame_error_i) begin
                sentence_bad_pulse_o <= 1'b1;
                state_q              <= ST_WAIT_DOLLAR;
            end

            if (byte_valid_i) begin
                // '$' provides immediate recovery from a truncated sentence.
                if (byte_i == 8'h24) begin
                    state_q            <= ST_HEADER;
                    header_pos_q       <= '0;
                    checksum_q         <= '0;
                    checksum_hi_q      <= '0;
                    field_index_q      <= '0;
                    field_char_index_q <= '0;
                    time_digit_count_q <= '0;
                    time_format_bad_q  <= 1'b0;
                    status_seen_q      <= 1'b0;
                    status_valid_q     <= 1'b0;
                    h_tens_q           <= '0;
                    h_ones_q           <= '0;
                    m_tens_q           <= '0;
                    m_ones_q           <= '0;
                    s_tens_q           <= '0;
                    s_ones_q           <= '0;
                end else begin
                    unique case (state_q)
                        ST_WAIT_DOLLAR: begin
                            // Ignore data until the next NMEA start marker.
                        end

                        ST_HEADER: begin
                            header_mismatch = 1'b0;
                            checksum_q <= checksum_q ^ byte_i;

                            unique case (header_pos_q)
                                3'd0: header_mismatch = (byte_i != 8'h47); // G
                                3'd1: header_mismatch = !((byte_i == 8'h50) ||
                                    ((ACCEPT_GN != 0) && (byte_i == 8'h4E))); // P or N
                                3'd2: header_mismatch = (byte_i != 8'h52); // R
                                3'd3: header_mismatch = (byte_i != 8'h4D); // M
                                3'd4: header_mismatch = (byte_i != 8'h43); // C
                                3'd5: begin
                                    header_mismatch = (byte_i != 8'h2C); // comma
                                    if (!header_mismatch) begin
                                        state_q            <= ST_DATA;
                                        field_index_q      <= 2'd1;
                                        field_char_index_q <= '0;
                                    end
                                end
                                default: header_mismatch = 1'b1;
                            endcase

                            // Other NMEA sentence types are normal traffic and
                            // are ignored rather than reported as bad data.
                            if (header_mismatch) begin
                                state_q <= ST_WAIT_DOLLAR;
                            end else if (header_pos_q != 3'd5) begin
                                header_pos_q <= header_pos_q + 1'b1;
                            end
                        end

                        ST_DATA: begin
                            if (byte_i == 8'h2A) begin // '*'
                                state_q <= ST_CHECKSUM_HI;
                            end else if ((byte_i == 8'h0D) || (byte_i == 8'h0A)) begin
                                sentence_bad_pulse_o <= 1'b1;
                                state_q              <= ST_WAIT_DOLLAR;
                            end else begin
                                checksum_q <= checksum_q ^ byte_i;

                                if (byte_i == 8'h2C) begin // comma
                                    if (field_index_q != 2'd3)
                                        field_index_q <= field_index_q + 1'b1;
                                    field_char_index_q <= '0;
                                end else begin
                                    // UTC field: HHMMSS with optional fraction.
                                    if (field_index_q == 2'd1) begin
                                        if (field_char_index_q < 6) begin
                                            if (is_digit(byte_i)) begin
                                                unique case (field_char_index_q)
                                                    0: h_tens_q <= byte_i - 8'h30;
                                                    1: h_ones_q <= byte_i - 8'h30;
                                                    2: m_tens_q <= byte_i - 8'h30;
                                                    3: m_ones_q <= byte_i - 8'h30;
                                                    4: s_tens_q <= byte_i - 8'h30;
                                                    5: s_ones_q <= byte_i - 8'h30;
                                                    default: ;
                                                endcase
                                                time_digit_count_q <= time_digit_count_q + 1'b1;
                                            end else begin
                                                time_format_bad_q <= 1'b1;
                                            end
                                        end else if ((field_char_index_q == 6) &&
                                                     (byte_i != 8'h2E)) begin
                                            time_format_bad_q <= 1'b1;
                                        end else if ((field_char_index_q > 6) &&
                                                     !is_digit(byte_i)) begin
                                            time_format_bad_q <= 1'b1;
                                        end
                                    end

                                    // RMC field 2: A = active, V = void.
                                    if ((field_index_q == 2'd2) &&
                                        (field_char_index_q == 0)) begin
                                        status_seen_q  <= 1'b1;
                                        status_valid_q <= (byte_i == 8'h41); // 'A'
                                    end

                                    if (field_char_index_q != 4'hF)
                                        field_char_index_q <= field_char_index_q + 1'b1;
                                end
                            end
                        end

                        ST_CHECKSUM_HI: begin
                            if (is_hex(byte_i)) begin
                                checksum_hi_q <= hex_value(byte_i);
                                state_q       <= ST_CHECKSUM_LO;
                            end else begin
                                sentence_bad_pulse_o <= 1'b1;
                                state_q              <= ST_WAIT_DOLLAR;
                            end
                        end

                        ST_CHECKSUM_LO: begin
                            if (is_hex(byte_i)) begin
                                checksum_low = hex_value(byte_i);
                                checksum_ok = (CHECK_CHECKSUM == 0) ||
                                              (checksum_q == {checksum_hi_q, checksum_low});
                                sentence_structure_ok = !time_format_bad_q &&
                                                        (time_digit_count_q == 3'd6) &&
                                                        time_range_valid && status_seen_q;

                                if (checksum_ok && sentence_structure_ok) begin
                                    if (!status_valid_q)
                                        fix_invalid_pulse_o <= 1'b1;

                                    if (status_valid_q || (REQUIRE_FIX_VALID == 0)) begin
                                        time_valid_pulse_o <= 1'b1;
                                    end
                                end else begin
                                    sentence_bad_pulse_o <= 1'b1;
                                end
                                state_q <= ST_WAIT_DOLLAR;
                            end else begin
                                sentence_bad_pulse_o <= 1'b1;
                                state_q              <= ST_WAIT_DOLLAR;
                            end
                        end

                        default: state_q <= ST_WAIT_DOLLAR;
                    endcase
                end
            end
        end
    end
endmodule

`default_nettype wire
