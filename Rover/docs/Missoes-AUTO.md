# Missões AUTO de frente e de ré

Como montar missões no Mission Planner para este rover (`rover.lua`: mixer em `modules/mixer.lua`, alinhamento em `modules/yaw_align.lua`, driver CAN em `modules/can_*.lua`).
Verificado no código do ArduRover 4.5 e dos scripts em 19/11/2026.

## Como o AUTO chega às rodas

1. O ArduRover calcula acelerador e direção (`vehicle:get_control_output`).
2. O mixer (`applyPWMAutoMode`) aplica o `THRO_INV` só no acelerador e mistura:
   `esquerda = t + s`, `direita = t − s`.
3. No CAN, o `esc_can` transforma em rpm alvo de cada lado.

**Ré funciona de forma nativa.** Na ré o ArduRover manda acelerador negativo e a direção continua com o mesmo
sinal: num skid-steer o giro vem só da diferença entre os lados, então a mesma mistura vale para frente e ré.
O próprio mixer do ArduPilot (`output_skid_steering`) também não inverte a direção na ré.

## Comando de ré: `DO_SET_REVERSE`

No Mission Planner: comando **DO_SET_REVERSE**, `P1 = 1` liga a ré, `P1 = 0` volta para frente.

- Ele vale para os **waypoints seguintes**: coloque-o **entre** o último waypoint de frente e o primeiro de ré.
- Na ré, o rover anda de costas até o próximo waypoint (a traseira aponta para o destino).
- `WP_SPEED`, `WP_RADIUS`, `WP_PIVOT_ANGLE` e o resto valem igual nos dois sentidos.

## Alinhamento de rumo pelo campo Alt (regra do mixer)

Quando o rover chega a um `WAYPOINT`, o **Alt** do waypoint define o que acontece:

| Alt | Efeito |
|---|---|
| `-1` | passa direto, continua em AUTO |
| outro valor (graus) | para, vai para STEERING, gira até esse rumo (frente do rover) e volta para AUTO |

O Mission Planner costuma preencher a altitude padrão (por exemplo 927,5): o rover então para e gira para
207,5°. **Coloque Alt = -1 em todo waypoint que não deve alinhar.**

## Regra crítica: alinhamento desliga a ré

Toda troca de modo no ArduRover faz `set_reversed(false)` (`Rover/mode.cpp`). O alinhamento troca
AUTO → STEERING → AUTO, e ao voltar o AUTO retoma o waypoint atual (`MIS_RESTART = 0`) **sem** repetir o
`DO_SET_REVERSE` que veio antes dele. Resultado: o trecho que deveria ser de ré é feito **de frente**.

**Correção de 19/11 (mixer):** ao entrar em AUTO, se houver um `DO_SET_REVERSE 1` logo antes do waypoint
atual, o mixer reinicia a partir dele e a ré volta (mensagem `AUTO: re restaurada no WP n`). Para isso cada
waypoint de ré precisa do seu próprio `DO_SET_REVERSE 1` logo antes. O `mission_editor.py` monta isso
sozinho pela opção **Chegar de ré**. Com esse formato o alinhamento antes da ré ou dentro dela funciona, e
o rumo alinhado é o da **frente** (para a traseira apontar o destino: rumo do trecho + 180).
Observação: ao terminar o alinhamento o mixer vai para HOLD; quem volta para AUTO é a rotina externa (ROS).

Sem esse formato (um único `DO_SET_REVERSE` para vários waypoints) continuam valendo as regras abaixo:

1. O **waypoint imediatamente antes de um `DO_SET_REVERSE`** precisa ter **Alt = -1**
   (o alinhamento dispara logo depois do `DO_SET_REVERSE` ter rodado e o apaga).
2. **Todos os waypoints dentro de um trecho de ré** precisam ter **Alt = -1**.
3. Alinhamento de rumo só em trecho de frente, e o trecho seguinte ao alinhamento começa de frente.
   Se precisar alinhar e depois dar ré, alinhe num waypoint, faça um waypoint curto de frente com Alt = -1 e
   só então coloque o `DO_SET_REVERSE`.
4. Não troque de modo pelo rádio no meio de um trecho de ré: ao voltar para AUTO o rover segue de frente.

Também: ao terminar a missão o mixer coloca o rover em MANUAL.

## Exemplos

### Ida de frente e volta de ré (sem girar no fim)

| # | Comando | P1 | Lat/Lon | Alt |
|---|---|---|---|---|
| 1 | WAYPOINT | | ponto A | -1 |
| 2 | WAYPOINT | | ponto B | -1 |
| 3 | DO_SET_REVERSE | 1 | | |
| 4 | WAYPOINT | | ponto A | -1 |
| 5 | DO_SET_REVERSE | 0 | | |

O rover vai de A até B de frente e volta para A de costas, sem pivotar em B.
Arquivo pronto (coordenadas do campo de teste de 18/11, A→B 10 m para leste): `exemplo-frente-re.waypoints`.
Antes de usar, confira no mapa se os pontos estão numa área livre e mova-os se preciso.

### Frente, alinha o rumo, depois ré

| # | Comando | P1 | Lat/Lon | Alt |
|---|---|---|---|---|
| 1 | WAYPOINT | | A | -1 |
| 2 | WAYPOINT | | B | 90 (para e gira para leste) |
| 3 | WAYPOINT | | C, 1–2 m à frente de B | -1 |
| 4 | DO_SET_REVERSE | 1 | | |
| 5 | WAYPOINT | | D (atrás) | -1 |
| 6 | DO_SET_REVERSE | 0 | | |

O waypoint 3 existe só para que o alinhamento do 2 aconteça antes do `DO_SET_REVERSE`.

## Editor de rumos

`python Rover/tools/mission_editor.py`: baixa a missão do rover, mostra cada waypoint com o efeito
(passa direto / alinha N graus), marca em vermelho os conflitos com a ré, edita o rumo numa bússola
(0 = norte, sentido horário) e envia de volta. Feche o `esc_monitor.py` antes (mesma porta UDP).

## Conferir no log

Com o `esc_monitor.py` gravando, na ré as quatro rodas trocam de sinal de rpm e `ESC_TGT_L/R` ficam com o sinal
oposto ao da ida. Se um trecho de ré aparecer de frente, procure no log uma troca de modo (`MODE`) logo antes.
