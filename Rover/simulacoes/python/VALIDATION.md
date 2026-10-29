# Validação — 28/10/2026

## Reorganização

Todos os fontes `.py` de `Rover` estão em `Rover/python/`. Os arquivos de
`FuncoesPython` ficaram no local anterior, conforme solicitado. Foram
atualizados os lançadores de sensores, preparação Lua e missões, os
caminhos de testes e a documentação. Os caminhos de dados anteriores
continuam resolvendo para `Rover/gazebo_simple/run` e
`Rover/mission_tune/runs`, mesmo com os fontes em outra pasta.

Verificações concluídas:

- Sintaxe dos fontes Python e dos lançadores shell alterados.
- Sete testes de projeção, exportação de mapa e preparação Lua.
- Vinte testes das ferramentas de missão.
- Preparação Lua pelo lançador atualizado em `--prepare-only`.
- Geração offline de missão pela chamada `mission_tune.sh`.
- Dependências dinâmicas do binário Hector Mapping local.

## Teste ROS com movimento sintético

`tests/validate_ros_slam.py` usou ROS master separado, uma nuvem sintética
com cobertura parcial de um ambiente conhecido e uma trajetória de 0,8 m
em X, 0,2 m em Y e 0,12 rad de giro. Somente a nuvem foi enviada ao SLAM;
a trajetória conhecida foi usada depois para comparar os resultados.

Na execução com os padrões finais de acumulação e resolução:

- 140 scans processados e mapa salvo.
- Erro de posição final de aproximadamente 0,005 m.
- Erro de rumo final de aproximadamente 0,004 rad.
- Leitura PointCloud2 e remoção de retornos zero conferidas.
- PGM com células livres, ocupadas e desconhecidas, acompanhado do YAML.

Esses números pertencem ao teste sintético e não representam a precisão
do MID360 no Gazebo ou no hardware. Evidência:
`run/validation/validation.json` e `run/validation/slam/`.

## MID360 no Gazebo

O mundo `gravel_plane` foi iniciado em ROS/Gazebo isolado, com os obstáculos,
um rover e somente o MID360 habilitado, sem SITL nem comandos de movimento.
O leitor usa o tópico `/rover_argo_N1/Instance1/livox/lidar` do tipo
`sensor_msgs/PointCloud`, com `frame_id=mid360_sensor`.

Inicialmente o sensor anunciava o tópico, mas não publicava sem GUI.
Foi acrescentado `<always_on>true</always_on>` ao sensor em:

`/home/grin/catkin_ws/src/rover-argo-gazebo/rover_argo_gazebo/urdf/components/mid360.xacro`

Após reiniciar o mundo, a leitura produziu NPZ e PNG. O SLAM recebeu 202
scans no ensaio final de 20 segundos, abriu a janela e salvou o mapa em
`run/gazebo_slam_final/`. A existência da janela foi conferida no X11.
O mapa final teve 273 células ocupadas, 14.620 livres e o restante
desconhecido, coerente com a cobertura parcial e o rover parado.

Sem acumulação, a baixa cobertura angular da faixa horizontal provocou
grande deriva. Foram adotadas cinco nuvens por scan e resolução de 0,1 m.
O teste parado ainda apresentou variação da pose estimada da ordem de
dezenas de centímetros. Não houve missão completa em movimento no Gazebo,
fechamento de laço ou teste no rover real. Para navegação, a configuração
precisa ser avaliada na trajetória e nas velocidades de uso.

Arquivos de evidência:

- `run/gazebo_mid360.npz`: primeira nuvem recebida.
- `run/gazebo_mid360.png`: visualização dos retornos próximos.
- `run/gazebo_slam_final/map.pgm`, `map.yaml`, `map_preview.png`.
- `run/gazebo_slam_final/trajectory.csv`, `status.json`, `hector.log`.

Os processos de teste foram encerrados. Nenhum teste armou o rover.
