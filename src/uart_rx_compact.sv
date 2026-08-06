//`default_nettype none

// Compact synthesizable 8-N-1 UART receiver.
// CLK_FREQ_HZ must match the frequency applied to clk_i.
module uart_rx_compact #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer BAUD_RATE   = 9_600
) (
    input  logic       clk_i,
    input  logic       rst_ni,
    input  logic       serial_i,
    output logic [7:0] data_o,
    output logic       data_valid_o,
    output logic       frame_error_o
);
    localparam integer CLKS_PER_BIT = CLK_FREQ_HZ / BAUD_RATE;
    localparam integer COUNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);
    localparam integer HALF_BIT = (CLKS_PER_BIT - 1) / 2;

    typedef enum logic [1:0] {
        ST_IDLE,
        ST_START,
        ST_DATA,
        ST_STOP
    } state_t;

    state_t state_q;
    logic serial_meta_q;
    logic serial_sync_q;
    logic [COUNT_W-1:0] clk_count_q;
    logic [2:0] bit_index_q;
    logic [7:0] data_q;

    // Synchronize the asynchronous UART input.
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            serial_meta_q <= 1'b1;
            serial_sync_q <= 1'b1;
        end else begin
            serial_meta_q <= serial_i;
            serial_sync_q <= serial_meta_q;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q       <= ST_IDLE;
            clk_count_q   <= '0;
            bit_index_q   <= '0;
            data_q        <= '0;
            data_o        <= '0;
            data_valid_o  <= 1'b0;
            frame_error_o <= 1'b0;
        end else begin
            data_valid_o  <= 1'b0;
            frame_error_o <= 1'b0;

            unique case (state_q)
                ST_IDLE: begin
                    clk_count_q <= '0;
                    bit_index_q <= '0;
                    if (!serial_sync_q)
                        state_q <= ST_START;
                end

                ST_START: begin
                    if (clk_count_q == HALF_BIT) begin
                        clk_count_q <= '0;
                        if (!serial_sync_q)
                            state_q <= ST_DATA;
                        else
                            state_q <= ST_IDLE; // False start bit.
                    end else begin
                        clk_count_q <= clk_count_q + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (clk_count_q == (CLKS_PER_BIT - 1)) begin
                        clk_count_q         <= '0;
                        data_q[bit_index_q] <= serial_sync_q;
                        if (bit_index_q == 3'd7) begin
                            bit_index_q <= '0;
                            state_q     <= ST_STOP;
                        end else begin
                            bit_index_q <= bit_index_q + 1'b1;
                        end
                    end else begin
                        clk_count_q <= clk_count_q + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (clk_count_q == (CLKS_PER_BIT - 1)) begin
                        clk_count_q <= '0;
                        if (serial_sync_q) begin
                            data_o       <= data_q;
                            data_valid_o <= 1'b1;
                        end else begin
                            frame_error_o <= 1'b1;
                        end
                        state_q <= ST_IDLE;
                    end else begin
                        clk_count_q <= clk_count_q + 1'b1;
                    end
                end

                default: state_q <= ST_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
