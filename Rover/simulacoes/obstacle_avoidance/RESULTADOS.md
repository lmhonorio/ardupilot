> Atualização: o perfil experimental de simulação agora habilita BendyRuler e a ponte transmite setores observados com cobertura parcial. O ensaio abaixo descreve a configuração anterior, que exigia 72 setores. A validação de missão, cobertura e parada por falha continua pendente.

# Resultados e limitações

## Evidências registradas

- A auditoria e cópia de parâmetros existentes estão em `AUDITORIA.md` e `parametros_mavproxy_antes.parm`. Esses parâmetros são o último snapshot do MAVProxy, não uma leitura comprovada em tempo real.
- Testes unitários verificaram frente, trás, esquerda, direita, setor não observado, solo, auto-retornos, valores não finitos e conversão para centímetros/ângulo horário.
- No ROS master isolado 11331, uma nuvem sintética legada passou pelo processador e ponte UDP 14597. Em SITL isolado `-I20`, sysid 21, com MAVProxy e `PRX1_TYPE=2`, o ArduPilot devolveu `DISTANCE_SENSOR`: frente 200 cm, direita 400 cm, trás 300 cm e esquerda 200 cm. As duas últimas distâncias de frente/esquerda diferem ligeiramente das coordenadas de entrada pelo deslocamento do MID360. Os parâmetros experimentais foram alterados apenas na EEPROM do SITL isolado.
- No Gazebo isolado, a nuvem real do plugin foi aceita. Uma amostra estacionária apresentou 17 de 72 setores com retorno válido após o filtro; não é uma medição de cobertura para todas as poses. Com a barreira padrão de 72 setores a ponte informou `COBERTURA_INSUFICIENTE=17/72; NAO_ENVIADO`.
- Ao encerrar o processador, a ponte informou `SCAN_DESATUALIZADO` e deixou de transmitir. Isso **não** demonstrou parada automática do ArduPilot.
- `start_obstacle_avoidance.sh`, `--status` e `--stop` foram exercitados no ROS master isolado.

## Ainda não concluído

Não houve ensaio A/B em AUTO, trajetória, teste de colisão ou comprovação de retomada. Os cenários adicionais e gráficos não foram executados. O mundo derivado `gravel_plane_avoidance_test` foi preparado, mas não foi usado em uma missão. Não foi comprovado se o comportamento anteriormente observado resultava de um outro mecanismo de desvio.

O firmware atual descarta leituras `65535` e não representa regiões desconhecidas como obstáculos. Portanto, o BendyRuler pode dirigir por setores sem medições se for ativado. O bloqueio de 72 setores usado naquele ensaio impediu o encaminhamento; agora o perfil experimental transmite com pelo menos um setor observado. Mesmo uma cobertura de 72 setores numa única altura não certificaria que todo o volume está livre. O LiDAR simulado tem cobertura variável, e não há compensação de movimento na projeção. A configuração experimental atual não resolve essa limitação; sem uma estratégia de cobertura e uma parada validada em caso de perda de percepção, o desvio não deve ser considerado confiável.

Não houve alteração do controle Lua, FAST-LIO, EKF, ganhos ou parâmetros do SITL do usuário. Não houve comando de armação nem movimento nos testes.
