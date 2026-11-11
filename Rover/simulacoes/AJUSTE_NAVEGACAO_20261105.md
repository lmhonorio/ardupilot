# Ajuste de navegação — Gazebo, 05/11/2026

O ajuste foi feito no controlador nativo do ArduRover. Na instância observada, `BATT_SOC_ALLOC=0`: o Lua de alocação estava desabilitado e não era ele que produzia as oscilações. O Lua e o receptor FAST-LIO foram preservados. Ativar a alocação Lua posteriormente é uma configuração diferente da validada aqui.

## O que foi corrigido

- Mais antecipação e menos integral no controle de giro: `ATC_STR_RAT_FF=0.4`, `ATC_STR_RAT_I=0.05`, com `ATC_STR_RAT_P=0.4`.
- Correção de posição mais rápida, com maior amortecimento de velocidade lateral: `PSC_POS_P=0.6`, `PSC_VEL_P=2`, `PSC_VEL_I=0`, `PSC_VEL_D=0`.
- Velocidade de missão de 0,6 m/s, mantendo `WP_RADIUS=0.3`. O raio de aceitação não foi aumentado para esconder as paradas.
- Giro no próprio eixo a até 30°/s (`WP_PIVOT_RATE`), com `WP_PIVOT_DELAY=2` s após atingir a tolerância angular do controlador. Esta é uma pausa intencional nos cantos, não uma falha de avanço. A taxa de 30°/s já estava persistida; agora está explícita nos perfis. O tempo de estabilização antes era zero.
- `ATC_STOP_SPEED=0.05`, definido uma única vez em cada perfil. Antes havia duas definições, 0,1 e 0,3. Esse parâmetro define quando o controlador considera a parada concluída; não é a velocidade mínima de navegação nem o raio do waypoint.
- `ATC_SPEED_I=1`, como estava efetivamente na EEPROM. Reduzi-lo para 0,2 em um dos ensaios piorou o acompanhamento da velocidade; a redução foi descartada.

Os valores estão em [`navegacao_gazebo.param`](../navegacao_gazebo.param), [`nroverobst.param`](../nroverobst.param) e [`nrover_sem_desvio.param`](../nrover_sem_desvio.param). São ajustes para este modelo no Gazebo, não uma calibração do rover físico.

## Como aplicar e repetir

Na instância usada nos ensaios, os valores foram gravados e conferidos pela comunicação MAVProxy. Após os testes, o SITL foi reaberto pelo `sim_1_gzrover.sh` em um terminal. Os 19 parâmetros do arquivo de ajuste foram conferidos novamente; rover em HOLD, desarmado e missão preservada. Em outra inicialização com EEPROM antiga, `--add-param-file` sozinho pode não substituí-los. Com o rover parado e desarmado, no **console MAVProxy**:

```text
param load /home/grin/ardupilot/Rover/navegacao_gazebo.param
param show PSC_POS_P
param show PSC_VEL_P
param show ATC_STR_RAT_FF
param show WP_SPEED
param show WP_PIVOT_DELAY
param show BATT_SOC_ALLOC
```

Esperado: `0.6`, `2`, `0.4`, `0.6`, `2`; a alocação era `0` nos ensaios. O arquivo de ajuste não muda a seleção da alocação Lua, a missão, sensores ou obstáculos. Confira estes últimos separadamente conforme o [tutorial](TUTORIAIS_SERVICOS_SEPARADOS.md).

Para repetir **a missão que estava carregada nesta sessão**, os waypoints de navegação começam no item 2:

```text
mode hold
wp list
wp set 2
arm throttle
mode auto
```

`mode hold` interrompe o movimento. Não use `-w`: isso apagaria a EEPROM. Os oito itens de missão foram preservados; nenhum upload de missão foi necessário nos ensaios.

## Medição

Os ensaios usam o mundo `gravel_plane`, o MID360 e as câmeras do rover, a mesma missão e o mesmo modelo. Entre tentativas, o SITL foi reiniciado e o modelo foi reposicionado na origem. GPS/EKF e rumo foram conferidos antes de armar; ainda existe variação de física e estimativa entre execuções.

Erro lateral: distância assinada à reta waypoint anterior → waypoint atual, usando a posição GPS/EKF, entre 10% e 90% de progresso dos segmentos 2→3, 3→4, 4→5 e 5→6. Erro de rumo: RMS da diferença angular para o segmento, entre 20% e 80%. O primeiro deslocamento, da origem até o waypoint 2, não entra nesses indicadores. Os tempos abaixo são de relógio real, com a carga gráfica e de sensores desta máquina.

| Ensaio | Concluído | Tempo (s) | Erro lateral RMS (m) | Erro de rumo RMS (°) | Velocidade mediana nas retas (m/s) |
| --- | --- | ---: | ---: | ---: | ---: |
| Referência, valores anteriores | sim | 256.0 | 0.653 | 27.0 | 0.37 |
| Candidato 1 | sim | 170.9 | 0.376 | 15.9 | 0.57 |
| Candidato 2 | sim | 153.4 | 0.406 | 8.1 | 0.55 |
| Candidato 3 | sim | 169.1 | 0.282 | 9.7 | 0.57 |
| Repetição candidato 3 | sim | 167.1 | 0.319 | 20.1 | 0.55 |
| Ajuste final | sim | 166.2 | 0.244 | 13.1 | 0.54 |
| Repetição ajuste final | sim | 169.4 | 0.295 | 14.7 | 0.58 |

| Segmento | Referência RMS (m) | Ajuste final RMS (m) | Repetição RMS (m) |
| --- | ---: | ---: | ---: |
| 2→3 | 0.898 | 0.149 | 0.360 |
| 3→4 | 0.495 | 0.105 | 0.280 |
| 4→5 | 0.564 | 0.375 | 0.274 |
| 5→6 | 0.180 | 0.129 | 0.075 |

As sete execuções concluíram os pontos. Nas duas últimas, não houve amostras com velocidade abaixo de 0,08 m/s enquanto restavam entre 0,3 e 1 m ao waypoint nos segmentos medidos; isso não garante ausência de paradas em outras condições. Houve transientes laterais de até aproximadamente 0.88 m. A redução de RMS não significa que todo o percurso ficou dentro do valor médio da reta.

![Trajetórias e comparação](../python/run/heading_20261105/comparacao.png)

O desvio continuou com os valores que estavam persistidos (`OA_TYPE=1`, `AVOID_ENABLE=1`), mesmo tendo sido escolhido o arquivo `nrover_sem_desvio.param`. Esses valores não foram desligados para favorecer as medições. Isso demonstra por que é necessário conferir a EEPROM. Estes ensaios de seguimento não constituem uma validação separada do desvio em colisões.

Os resultados mostram melhora medida, mas não erro zero nem prova de ótimo global. Trechos próximos aos cantos têm transientes; GPS/EKF e atrito do modelo limitam a precisão. Os logs estão em [`python/run/heading_20261105`](../python/run/heading_20261105/), incluindo missão baixada, parâmetros anteriores, telemetria, amostras, resultados por trecho e os scripts de ensaio. O backup prévio da EEPROM e dos perfis está em [`backups/heading_20261105`](../backups/heading_20261105/).

## MID360 e docking

Foi encontrado `gzserver` encerrado com código 139. O ROS ainda listava os tópicos, mas não havia atualização de `/clock`, LiDAR ou IMU. O servidor e o modelo foram reiniciados, e o MID360 voltou a cerca de 10 Hz de tempo simulado.

A rotina LiDAR agora diferencia relógio parado, ausência de nuvem, timestamp inválido e frame incorreto, antes de assumir o modo do rover. Para conferir apenas a recepção, sem movimento:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
/usr/bin/python3.8 python/docking_lidar/entrar_lidar.py --diagnostico
```

Ver [diagnóstico e comandos do docking LiDAR](docking/DOCKING_LIDAR.md). A verificação da nuvem não equivale à validação da entrada completa; as manobras de docking não foram executadas nestes ensaios de navegação.

Na checagem final de 05/11/2026, com o rover a cerca de 4,3 m do centro da estação, a nuvem chegou normalmente, mas continha somente 13–27 pontos na faixa de altura das paredes por mensagem. O ajuste geométrico recusou as três amostras: pontos insuficientes. Portanto, **a recepção foi recuperada, mas a aquisição geométrica nessa posição ainda não foi validada**. O diagnóstico de recepção não libera a manobra e os limites de geometria não foram afrouxados.
