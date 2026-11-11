# Mapa 3D, MID360 e docking

Para usar também **GLIM com otimização global e fechamento de laços**, consulte [GLIM com MID360](GLIM_MID360.md). Ele possui mapa e referenciais próprios; o fluxo abaixo continua sendo a fonte local do docking.

O fluxo padrão agora usa **XYZ completo do MID360 → FAST-LIO → mapa de voxels 3D → nuvem local recente → ajuste das superfícies do contêiner**. O início integrado `./iniciar_simulacao_completa.sh` já abre esse fluxo e o RViz configurado. O Hector 2D permanece como ferramenta opcional, fora do docking LiDAR.

O FAST-LIO fornece odometria LiDAR/IMU e mapeamento 3D; este conjunto não adiciona fechamento de laços. A origem `camera_init` nasce quando ele inicia. Reiniciar FAST-LIO muda essa referência e exige refazer o mapa; o receptor Lua recebe uma nova sessão.

## Se Gazebo e SITL já estiverem abertos

Mantenha o rover parado durante a inicialização do FAST-LIO. Encerre os processos anteriores de FAST-LIO e de mapa com Ctrl+C antes de iniciar as novas versões. Não execute os monitores QR e LiDAR ao mesmo tempo na porta 14551.

Em **cada terminal**, prepare o ambiente:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/ardupilot/Rover
```

1. Terminal FAST-LIO:

   ```bash
   /usr/bin/python3.8 python/slam/fast_lio/run_fast_lio.py
   ```

2. Terminal mapa e nuvem local 3D:

   ```bash
   /usr/bin/python3.8 python/slam/slam_mid360_3d.py
   ```

3. Terminal visualização:

   ```bash
   rviz -d python/slam/mid360_3d.rviz
   ```

   O referencial fixo é `camera_init`. O mapa aparece colorido pela altura; a geometria local aparece amarela. Marque **MID360 XYZ corrigido** para mostrar também a varredura instantânea. O mouse permite girar a vista em 3D.

4. Confira a recepção, sem movimento nem conexão ao MAVProxy:

   ```bash
   /usr/bin/python3.8 python/docking_lidar/entrar_lidar.py --diagnostico
   rostopic echo -n 1 /rover_slam_3d/status
   ```

   `frames` deve crescer e `error` ficar vazio. O diagnóstico de recepção não certifica que o contêiner foi reconhecido.

5. Para **entrar**, em um terminal separado:

   ```bash
   /usr/bin/python3.8 python/docking_lidar/entrar_lidar.py
   ```

   O programa espera GUIDED, rover armado e distância inferior a 8 m da estação cadastrada. Solicita HOLD para confirmar a geometria, depois conduz a aproximação e entrada. As condições de parada/alinhamento permanecem ativas.

6. Para **sair**, somente depois de estacionar dentro e estar em HOLD:

   ```bash
   /usr/bin/python3.8 python/docking_lidar/sair_lidar.py
   ```

   A saída exige pose dentro da região central, alinhamento e FAST-LIO válido. Ela não faz a aproximação ao contêiner. Consulte os limites e o comportamento em [Docking LiDAR](docking/DOCKING_LIDAR.md).

## Tópicos e arquivos

| Tópico | Conteúdo | Referencial |
| --- | --- | --- |
| `/rover_argo_N1/Instance1/livox/lidar` | Varredura XYZ original, `PointCloud` | `mid360_sensor` |
| `/mid360/points3d` | XYZ corrigido para a origem dos raios, `PointCloud2` | `mid360_sensor` |
| `/fast_lio/cloud_registered` | Varredura 3D registrada no mundo local | `camera_init` |
| `/fast_lio/cloud_registered_body` | Varredura densa no referencial da IMU | `body` |
| `/fast_lio/odometry` | Posição XYZ e orientação, com covariância | `camera_init` → `body` |
| `/rover_slam_3d/map` | Mapa 3D acumulado, até 250 mil voxels | `camera_init` |
| `/rover_slam_3d/local_cloud` | Últimos 1,5 s registrados e transformados ao corpo atual | `body` |

Ao encerrar o mapa com Ctrl+C, ele exporta **`mapa.pcd`** em `python/run/slam_3d/<data>/`, com X, Y e Z em metros, e `status.json`. Não há imagem PGM/YAML 2D neste fluxo. A opção `--output-dir` escolhe a pasta; `--window` ajusta a janela local entre 0,2 e 3 s e `--voxel` a resolução do mapa. O padrão é 4 cm no mapa e 2,5 cm na nuvem local.

## Como a escassez de pontos foi tratada

Antes, o docking tomava uma só varredura, cortava uma faixa de altura e descartava Z. Agora o FAST-LIO publica a nuvem densa e o mapa reúne várias varreduras em um referencial comum. A nuvem local é transformada à pose atual da IMU, compensando translação e rotação; os pontos não são simplesmente empilhados no referencial móvel do sensor.

O ajuste usa retângulos finitos 3D para fundo, laterais, teto e piso. Fundo e pelo menos uma lateral precisam ter extensão horizontal e vertical suficiente. Piso isolado, parede única, poucos pontos ou ajuste ruim continuam sendo recusados. A manobra é planar, própria do rover em piso nivelado: estima X/Y/yaw e altura do centro quando há suporte de teto/piso. Sem esse suporte, a altura usa a referência conhecida da base; isso é informado em `height_observed`.

A configuração padrão representa o interior livre de 1 × 1 × 1 m, base do rover a aproximadamente 0,15 m do piso e MID360 em `[-0.003, 0, 0.434]` m em relação à base. A correção radial de 0,10 m é específica do plugin Gazebo usado aqui. Não aplique essa conversão a um MID360 físico.

O mapa permanente serve à visualização/exportação. O docking usa somente a nuvem **recente**; ausência de atualização, mudança de relógio, reinício de odometria, saltos de pose e covariâncias inválidas invalidam o buffer. A distância GPS e o rumo continuam sendo usados como semente para selecionar a estação cadastrada; o ajuste final usa os pontos LiDAR.

## Interpretação da mensagem de saída

Na pose enviada (`forward=0.592`, `right=0.140`, `normal_yaw=-0.184`), a distância longitudinal ao centro é aproximadamente **0,608 m**. A abertura está a 0,50 m do centro: essa pose não confirma estacionamento central. A mensagem vinha da verificação de **saída**, cuja região aceita vai até 0,30 m do centro. Ela agora informa as distâncias e orienta a usar `entrar_lidar.py` para aproximar/entrar. A tolerância de saída não foi ampliada.

## Validação de 05/11/2026

- 23 testes automáticos: geometria 3D, superfícies insuficientes, transformação de nuvens com movimento, expiração, salto de pose, relógio, conversão de eixos e mensagens FAST-LIO.
- Dez amostras estáticas perto do contêiner, na simulação do usuário: geometria confirmada nas dez.
- Dez amostras a aproximadamente 4,3 m, em Gazebo isolado sem controle do rover principal: geometria confirmada nas dez, com 403–455 pontos ajustados. Maior erro longitudinal observado ≈ 4,6 mm; lateral ≈ 2,3 cm; angular ≈ 0,31°.

A posição real do Gazebo foi usada **apenas para comparar o resultado dos testes**; não entra no programa de docking ou no mapa. Os ensaios são estáticos e não demonstram precisão durante a entrada completa, resistência a objetos móveis ou deriva em trajetos longos. As manobras completas não foram executadas nesta validação.

Registros: `python/run/slam_3d_validation/`. Backup anterior: `backups/slam_3d_20261105/`.
