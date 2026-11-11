> Atualização posterior: veja os [sete ensaios com ajuste aplicado](AJUSTE_NAVEGACAO_20261105.md). Este arquivo registra a investigação anterior.

# Ensaios de seguimento da missão armazenada no Rover

Em 05/11/2026 foi feita uma cópia de `Rover/eeprom.bin` em `Rover/python/run/linha_missao/sitl/`; a missão original não foi modificada. A cópia continha oito itens, com waypoints geográficos 2 a 6 e um `NAV_GUIDED_ENABLE` no item 7. O Gazebo, ROS master e SITL foram isolados da sessão principal, usando instância 20 e MAVProxy separado. O MID360 e o desvio estavam desativados e as verificações de armamento foram desabilitadas **somente nesse SITL isolado**. Logo, estes ensaios não validam desvio de obstáculos ou armamento no rover principal.

O Lua de orientação final da alteração anterior foi revertido para a versão de alocação diferencial simples, que repassa direção e velocidade do ArduRover. A versão removida está em `Rover/backups/antes_reverter_heading_20261105/`. Uma correção adicional proporcional ao erro transversal foi criada somente no diretório de testes e não foi instalada no rover, pois piorou o resultado no ensaio de um trecho.

O erro é a distância horizontal estimada pelo ArduPilot à reta que liga os dois waypoints, considerando amostras com projeção entre 10% e 90% do segmento. São valores RMS da posição estimada por GPS/EKF, não medições do contato das rodas com a linha:

| Ensaio | Trecho 2→3 | Trecho 3→4 | Trecho 4→5 | Resultado |
| --- | ---: | ---: | ---: | --- |
| Missão completa, alocação simples, `PSC_POS_P=0,2` | 0,43 m | 0,40 m | 0,38 m | Concluída, 298 s |
| Trecho 2→3 reiniciado, alocação simples | 0,53 m | — | — | Chegou ao waypoint 3 |
| Mesmo trecho reiniciado, correção Lua candidata | 1,46 m | — | — | Chegou; correção rejeitada |
| Mesmo trecho reiniciado, `WP_PIVOT_ANGLE=30` | 0,79 m | — | — | Chegou; mudança rejeitada |
| Mesmo trecho reiniciado, `PSC_POS_P=0,5` | 0,32 m | — | — | Chegou ao waypoint 3 |
| Missão completa, `PSC_POS_P=0,5` | 0,38 m | 0,49 m | 0,57 m | Concluída, 271 s |

Os reinícios de trecho usaram aproximadamente a mesma posição e orientação física, mas houve variação do rumo estimado pelo EKF. Nos ensaios completos, a orientação estimada registrada no início da missão também diferiu: aproximadamente 90° e 39°. Portanto os números não são uma comparação perfeitamente controlada e o ganho 0,5 não foi aplicado à configuração do usuário. As curvas também geram transientes; o algoritmo nativo não promete erro zero.

No primeiro ensaio, o rover ficou quase parado a aproximadamente 0,42 m do waypoint 3 antes de finalmente alcançá-lo. O raio configurado era 0,30 m e `ATC_STOP_SPEED` era 0,30 m/s, porém o registro não prova que esse parâmetro tenha sido a única causa. Não foi alterado na configuração do usuário.

Resultados e telemetria estão sob `Rover/python/run/linha_missao/`: `missao_original.json` e, para cada ensaio, `resultado.json` e `telemetry.jsonl`. O arquivo `avaliar.py` desse diretório foi usado para medir os trechos. A obtenção de trajetórias mais próximas da reta requer novos ensaios com orientação inicial controlada, incluindo o LiDAR e o planejador de obstáculos; não se deve inferir que os valores testados sejam ótimos.
