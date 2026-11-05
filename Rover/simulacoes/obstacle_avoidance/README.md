# Percepção experimental de obstáculos OLH-E

Este módulo implementa a projeção e a ponte ROS → MAVProxy → ArduPilot. **Não é ainda um sistema de desvio validado.** No teste do MID360 simulado houve apenas 17/72 setores com retorno; o ArduPilot ignora os setores desconhecidos. A ponte experimental agora transmite os setores observados mesmo com cobertura incompleta; o ArduPilot ignora os desconhecidos. Não foi implementada parada automática do rover em caso de falha. Não use esta versão para navegação autônoma sem supervisão ou com hardware real. Veja [resultados e pendências](RESULTADOS.md) e a [auditoria](AUDITORIA.md).

## Abrir a percepção

Inicie o cenário e SITL existentes conforme o [passo a passo](../PASSO_A_PASSO_ROS_FAST_LIO.md). Em outro terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
./start_obstacle_avoidance.sh
./start_obstacle_avoidance.sh --status
```

Veja o log em `python/run/obstacle_avoidance/module.log`. A ponte espera uma saída local **14557** do MAVProxy; 14550, 14555 e 14556 continuam destinadas às ferramentas existentes. O lançador completo prepara um perfil SITL separado com BendyRuler ativo; iniciar apenas este módulo não altera parâmetros do ArduPilot. Quando não houver saída MAVProxy nessa porta, indicará `SEM_MAVPROXY`. Para encerrar somente os dois processos deste módulo:

```bash
./start_obstacle_avoidance.sh --stop
```

Como alternativa para uso direto pelo roslaunch:

```bash
export ROS_PACKAGE_PATH="$HOME/ardupilot/Rover/python:$ROS_PACKAGE_PATH"
roslaunch olhe_obstacle_avoidance obstacles.launch
```

Não execute as duas formas simultaneamente. O wrapper verifica nós existentes; o roslaunch deve ser iniciado apenas uma vez.

## Visualizar e diagnosticar

```bash
rostopic echo /olhe/obstacles/status
rostopic echo /olhe/obstacles/bridge_status
rostopic echo -n 1 /olhe/obstacles/scan
```

`scan` é um LaserScan de 72 setores de 5 graus, em metros, com `NaN` para ausência de retorno; começa à frente e avança no sentido anti-horário ROS. `/olhe/obstacles/markers` mostra raios dos setores com retorno em `base_link`. No RViz, adicione LaserScan `/olhe/obstacles/scan` e Marker `/olhe/obstacles/markers`; escolha um Fixed Frame conectado ao frame do rover pelo TF existente. O programa não publica TF novo. Se o frame `base_link` não estiver conectado ao mapa ou ao `camera_init` na instalação atual, use `base_link` como Fixed Frame para visualizar os obstáculos locais.

A ponte converte para ângulos horários e centímetros, marca desconhecidos com 65535, usa MAVLink 2 `OBSTACLE_DISTANCE` em `MAV_FRAME_BODY_FRD` e envia somente dados recentes; fonte do receptor neste firmware: `libraries/AP_Proximity/AP_Proximity_MAV.cpp`. `STALE_LIDAR`, `SCAN_DESATUALIZADO`, `SEM_MAVPROXY` e `COBERTURA_INSUFICIENTE` são diagnósticos, não comandos de parada. O receptor no ArduPilot usa a orientação corporal esperada, mas não verifica o campo `frame` desta mensagem.

O filtro parte da montagem simulada fixa (-0,003, 0, 0,434) m, sem transformação TF concorrente, e aceita PointCloud legado e PointCloud2 no referencial do LiDAR. A faixa vertical e caixa de exclusão próprias em `Rover/python/obstacle_avoidance/config.yaml` precisam ser validadas contra o veículo e todos os sensores instalados. Não se acumula varreduras em movimento, pois não há compensação de pose. `voxel_size_m` está reservado e não é aplicado; a janela temporal é usada como limite de validade. Isso evita fazer médias que apaguem retornos próximos e evita juntar nuvens sem compensar movimento.

## Preparação futura de testes em SITL isolado

O mundo `gravel_plane_avoidance_test` é uma cópia do mundo existente, com uma caixa em x=7,5 m no trajeto x=0 a x=15 m; a docking station continua no mundo de teste, deslocada para não interferir. Ele ainda **não foi validado em missão**. `Rover/python/obstacle_avoidance/bendyruler_sim.parm` contém o perfil experimental de simulação: `PRX1_TYPE=2`, `OA_TYPE=1`, lookahead 5 m e margem 1 m, com reinicialização necessária. O lançador completo aplica esse perfil a uma EEPROM separada de simulação. A cobertura e a parada por falha ainda não foram resolvidas; os testes devem ser supervisionados. O arquivo não muda ATC, motores, Lua ou EKF.

Nos testes isolados, a saída MAVProxy dedicada e `mavfwd true` foram usadas para comunicar com o autopiloto, sem tocar na porta de física Gazebo. Não existe mecanismo automático de restaurar ou alterar parâmetros no seu SITL atual.
