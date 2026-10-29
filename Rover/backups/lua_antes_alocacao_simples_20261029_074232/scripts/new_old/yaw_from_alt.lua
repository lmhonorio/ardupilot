-- yaw_from_alt.lua
-- Workaround para Rovers: Usa a altitude do Waypoint como alvo de Yaw.
-- Ao chegar no WP, se Alt != 0, gira o rover para esse ângulo e aguarda o tempo P1.

local MODE_AUTO = 10
local MODE_GUIDED = 15
local MODE_HOLD = 4 

local WP_ID_WAYPOINT = 16

local TOLERANCE_M = 1.0  -- Distância para considerar que chegou no WP (metros)
local YAW_ERROR_MAX = 5  -- Erro máximo de yaw para considerar atingido (graus)
local TURN_SPEED_DEFAULT = 30 -- Se não definido, usa isso (graus/s) - *Usado se set_desired_turn_rate_and_speed não estiver disponível ou for usado controle manual*

-- Constantes para controle
local KH_STEERING = 0.05 -- Ganho P para steering

-- Estado
local STATE_MONITOR = 0
local STATE_TURNING = 1
local STATE_WAITING = 2

local current_state = STATE_MONITOR
local target_yaw_deg = 0
local wait_start_time = 0
local wait_duration_s = 0
local handling_wp_index = -1

function update()
    if not arming:is_armed() then
        current_state = STATE_MONITOR
        return update, 1000
    end

    local mode = vehicle:get_mode()
    local wp_dist = vehicle:get_wp_distance_m()
    local current_index = mission:get_current_nav_index()

    -- Máquina de Estados
    if current_state == STATE_MONITOR then
        -- Só monitora se estiver em AUTO
        if mode == MODE_AUTO then
            -- Se estamos perto de um WP e aidna não lidamos com ele
            if wp_dist and wp_dist < TOLERANCE_M and current_index ~= handling_wp_index then
                -- Lê o comando da missão
                local item = mission:get_item(current_index)
                if item and item:command() == WP_ID_WAYPOINT then
                     -- Verifica se tem Altitude ("Yaw")
                     local alt_val = item:z() -- Altitude é Z
                     if alt_val and math.abs(alt_val) > 0.001 then
                        -- Sim, esse WP requer giro especial!
                        target_yaw_deg = alt_val
                        wait_duration_s = item:param1() -- P1 é o tempo de espera
                        if not wait_duration_s then wait_duration_s = 0 end
                        
                        -- Inicia manobra
                        gcs:send_text(6, string.format("YawWorkaround: Girando para %.1f deg", target_yaw_deg))
                        
                        if vehicle:set_mode(MODE_GUIDED) then
                             current_state = STATE_TURNING
                             handling_wp_index = current_index
                             -- Reseta timers se necessário
                        else
                             gcs:send_text(4, "YawWorkaround: Falha ao trocar para GUIDED")
                        end
                     else
                        -- WP normal (sem Alt), marca como lidado para não checar de novo
                        handling_wp_index = current_index
                     end
                end
            end
        end

    elseif current_state == STATE_TURNING then
        -- Estamos em Guided, tentando girar
        if mode ~= MODE_GUIDED then
            -- Se o usuário mudou o modo, aborta
            current_state = STATE_MONITOR
            return update, 200
        end

        local current_yaw = math.deg(ahrs:get_yaw())
        if current_yaw < 0 then current_yaw = current_yaw + 360 end
        
        -- Normaliza target
        local t_yaw = target_yaw_deg
        if t_yaw < 0 then t_yaw = t_yaw + 360 end

        -- Erro
        local err = t_yaw - current_yaw
        if err > 180 then err = err - 360 end
        if err < -180 then err = err + 360 end

        if math.abs(err) < YAW_ERROR_MAX then
            -- Chegamos
            gcs:send_text(6, "YawWorkaround: Angulo atingido. Aguardando...")
            wait_start_time = millis()
            current_state = STATE_WAITING
            -- Para o veículo
            vehicle:set_steering_and_throttle(0, 0) 
        else
            -- Aplica steering
            -- Simples controle P para steering (-1 a 1)
            local steer = err * KH_STEERING
            if steer > 1 then steer = 1 end
            if steer < -1 then steer = -1 end
            
            -- Tenta girar no próprio eixo (skid steering)
            -- Thottle 0, Steering ativa rotação diferencial nos rovers skid
            -- Nota: Se for Ackermann, precisa de throttle para girar. 
            -- Assumindo Skid/Differential drive conforme contexto anterior ("pivot")
            vehicle:set_steering_and_throttle(steer, 0) 
        end

    elseif current_state == STATE_WAITING then
         if mode ~= MODE_GUIDED then
            current_state = STATE_MONITOR
            return update, 200
        end
        vehicle:set_steering_and_throttle(0, 0) -- Mantém parado

        local elapsed = (millis() - wait_start_time) / 1000.0
        if elapsed >= wait_duration_s then
            -- Tempo acabou, voltar para Auto e Próximo WP
            gcs:send_text(6, "YawWorkaround: Tempo esgotado. Voltando para AUTO.")
            
            -- Avança missão
            mission:set_current_cmd(current_index + 1)
            
            if vehicle:set_mode(MODE_AUTO) then
                current_state = STATE_MONITOR
                -- handling_wp_index continua o mesmo até chegar no próximo e o index mudar
            else
                gcs:send_text(4, "YawWorkaround: Falha ao voltar para AUTO")
                current_state = STATE_MONITOR -- Reseta para tentar recuperar
            end
        end
    end

    return update, 100 -- 10Hz
end

gcs:send_text(6, "YawFromAlt Script Carregado")
return update()
