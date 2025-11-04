// =======================================================
// FSM Cerradura digital 4 dígitos
// =======================================================
module digital_lock_fsm (
    input  logic       clk,
    input  logic       rst,
    input  logic       tick_1s,
    input  logic [3:0] digit_in,         // dígito desde switches
    input  logic       btn_capturar,     // capturar dígito
    input  logic       btn_confirmar,    // confirmar contraseña
    input  logic       btn_limpiar,      // limpiar entrada
    output state_t     estado,           // usa el typedef global
    output logic [1:0] intentos_usados,  // 0..3
    output logic [1:0] bloqueo_restante, // 0..3 (segundos de bloqueo)
    output logic [1:0] idx_digito,       // 0..3
    output logic [3:0] dig0,
    output logic [3:0] dig1,
    output logic [3:0] dig2,
    output logic [3:0] dig3
);

    // Registros internos
    state_t estado_r, estado_n;

    logic [1:0] intentos_r, intentos_n;
    logic [1:0] bloqueo_r,  bloqueo_n;
    logic [1:0] idx_r,      idx_n;
    // NUEVO: contador de dígitos realmente capturados (0..4)
    logic [2:0] count_r,    count_n;

    logic [3:0] d0_r, d0_n;
    logic [3:0] d1_r, d1_n;
    logic [3:0] d2_r, d2_n;
    logic [3:0] d3_r, d3_n;

    // Contraseña correcta: 1-2-3-4
    localparam logic [3:0] P0 = 4'd1;
    localparam logic [3:0] P1 = 4'd2;
    localparam logic [3:0] P2 = 4'd3;
    localparam logic [3:0] P3 = 4'd4;

    // Flanco de segundo para decrementar bloqueo
    logic tick_1s_r;
    logic tick_1s_rise;

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            tick_1s_r <= 1'b0;
        else
            tick_1s_r <= tick_1s;
    end
    assign tick_1s_rise = tick_1s & ~tick_1s_r;

    // ------------------------------
    // Estado secuencial
    // ------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            estado_r   <= ST_CERRADO;
            intentos_r <= 2'd0;
            bloqueo_r  <= 2'd0;
            idx_r      <= 2'd0;
            count_r    <= 3'd0;
            d0_r       <= 4'd0;
            d1_r       <= 4'd0;
            d2_r       <= 4'd0;
            d3_r       <= 4'd0;
        end else begin
            estado_r   <= estado_n;
            intentos_r <= intentos_n;
            bloqueo_r  <= bloqueo_n;
            idx_r      <= idx_n;
            count_r    <= count_n;
            d0_r       <= d0_n;
            d1_r       <= d1_n;
            d2_r       <= d2_n;
            d3_r       <= d3_n;
        end
    end

    // ------------------------------
    // Lógica combinacional
    // ------------------------------
    always_comb begin
        // Valores por defecto (mantener)
        estado_n   = estado_r;
        intentos_n = intentos_r;
        bloqueo_n  = bloqueo_r;
        idx_n      = idx_r;
        count_n    = count_r;
        d0_n       = d0_r;
        d1_n       = d1_r;
        d2_n       = d2_r;
        d3_n       = d3_r;

        // Limpieza de entrada
        if (btn_limpiar) begin
            idx_n   = 2'd0;
            count_n = 3'd0;
            d0_n    = 4'd0;
            d1_n    = 4'd0;
            d2_n    = 4'd0;
            d3_n    = 4'd0;
        end

        // Captura de dígitos (máx 4) // ignoramos cuando está bloqueado
        if (btn_capturar && (estado_r != ST_BLOQUEO)) begin
            if (count_r < 3'd4) begin
                case (idx_r)
                    2'd0: begin
                        d0_n = digit_in;
                        idx_n = 2'd1;
                    end
                    2'd1: begin
                        d1_n = digit_in;
                        idx_n = 2'd2;
                    end
                    2'd2: begin
                        d2_n = digit_in;
                        idx_n = 2'd3;
                    end
                    2'd3: begin
                        d3_n = digit_in;
                        // idx_n se queda en 3 (ya en el último dígito)
                    end
                    default: ;
                endcase
                count_n = count_r + 3'd1; // hemos capturado un dígito más
            end
            // si count_r == 4, se ignoran capturas extra
        end

        // Lógica de bloqueo: countdown en ST_BLOQUEO
        if (estado_r == ST_BLOQUEO && tick_1s_rise) begin
            if (bloqueo_r > 0)
                bloqueo_n = bloqueo_r - 1;
            if (bloqueo_r == 2'd1) begin
                // Al llegar a 0, regresamos a CERRADO
                estado_n   = ST_CERRADO;
                intentos_n = 2'd0;
                idx_n      = 2'd0;
                count_n    = 3'd0;
                d0_n       = 4'd0;
                d1_n       = 4'd0;
                d2_n       = 4'd0;
                d3_n       = 4'd0;
            end
        end

        // Confirmar contraseña
        if (btn_confirmar) begin
            case (estado_r)
                ST_CERRADO: begin
                    // Solo validamos si YA ingresó los 4 dígitos reales
                    if (count_r == 3'd4) begin
                        if ((d0_r == P0) &&
                            (d1_r == P1) &&
                            (d2_r == P2) &&
                            (d3_r == P3)) begin
                            // Contraseña correcta
                            estado_n   = ST_ABIERTO;
                            intentos_n = 2'd0;
                        end else begin
                            // Incorrecta
                            if (intentos_r == 2'd2) begin
                                // 3er intento => bloqueo
                                estado_n   = ST_BLOQUEO;
                                bloqueo_n  = 2'd3; // 3 segundos
                                intentos_n = 2'd0;
                            end else begin
                                intentos_n = intentos_r + 1;
                            end
                        end
                    end
                end

                ST_ABIERTO: begin
                    // Si confirma estando abierto, se cierra
                    estado_n   = ST_CERRADO;
                    intentos_n = 2'd0;
                    idx_n      = 2'd0;
                    count_n    = 3'd0;
                    d0_n       = 4'd0;
                    d1_n       = 4'd0;
                    d2_n       = 4'd0;
                    d3_n       = 4'd0;
                end

                ST_BLOQUEO: begin
                    // Mientras está bloqueado, ignoramos confirmaciones
                end

                default: ;
            endcase
        end
    end

    // Salidas
    assign estado           = estado_r;
    assign intentos_usados  = intentos_r;
    assign bloqueo_restante = bloqueo_r;
    assign idx_digito       = idx_r;
    assign dig0             = d0_r;
    assign dig1             = d1_r;
    assign dig2             = d2_r;
    assign dig3             = d3_r;

endmodule