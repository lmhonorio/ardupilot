# Validação do alinhamento e da velocidade de entrada — ROS 2

Em 09/11/2026, a revisão completou **seis entradas consecutivas no Gazebo
Harmonic**, incluindo deslocamentos laterais, rumos diferentes e repetição.
Todas terminaram em HOLD, desarmadas, com os oito itens da missão preservados e
os parâmetros temporários restaurados. Também passaram **97 testes de software**.

O maior erro de posição ao centro foi **3.47 cm** e o maior erro de
orientação foi **1.95°**, medidos independentemente pelo Gazebo.
São resultados destes ensaios, não garantia de precisão para qualquer posição
inicial, modelo, iluminação ou veículo real.

## Resultados medidos

Centro do contêiner: `(x=5,000; y=−2,000)` m; abertura em `x=4,5 m`.
Coordenadas e yaw da tabela pertencem ao mundo Gazebo (ENU, yaw zero para +x).
Os erros longitudinais negativos indicam parada antes do centro. A duração é o
tempo de execução do controlador; exclui o preparo e a estabilização do ensaio.

| Caso | Início (x; y; yaw) | Erro longitudinal [cm] | Erro lateral [cm] | Erro angular [°] | Duração [s] |
| --- | --- | ---: | ---: | ---: | ---: |
| Alinhado | (3.2; -2.0; 0°) | -3.11 | 0.01 | 0.00 | 42.9 |
| Lateral +0,5 m / +20° | (2.0; -1.5; 20°) | -1.65 | -0.47 | 1.95 | 77.4 |
| Lateral −0,5 m / −20° | (2.0; -2.5; -20°) | -1.00 | -0.92 | -1.90 | 85.8 |
| Rumo +25° | (2.5; -2.1; 25°) | -2.81 | -2.01 | 0.75 | 64.2 |
| Rumo −25° | (2.5; -1.9; -25°) | -2.95 | 1.84 | 0.25 | 70.8 |
| Repetição alinhada | (3.2; -2.0; 0°) | -3.06 | -0.31 | 0.70 | 52.5 |

![Trajetórias e posições finais](../../python/run/docking_validation/20261109_181935/trajetorias.png)

Registros completos: [resultados JSON](../../python/run/docking_validation/20261109_181935/results.json),
[versões dos arquivos por SHA-256](../../python/run/docking_validation/20261109_181935/source_sha256.json)
e [testes de software](../../python/run/docking_validation/20261109_181935/testes_software.txt).
Cada subdiretório contém console, trajetória real, telemetria MAVLink e missão
antes/depois. Os logs visuais ficam em `python/run/docking/<data>/`.

## Mudanças verificadas

- A câmera acompanha o QR com pan/tilt. A transformação usa os ângulos medidos
  das juntas no instante da imagem, com interpolação e rejeição de dados antigos.
- Após decodificar o QR cadastrado, o alinhamento com sua textura completa refina
  os cantos. A pose considera a parede vertical e usa uma referência angular
  inicial fraca do cadastro. O filtro mantém a orientação da parede em um
  referencial fixo; sua correção é atualizada com o rover parado.
- A aproximação converge para um ponto cerca de **1,2 m antes do centro**,
  ainda fora da abertura. O controlador interrompe a translação para corrigir
  o rumo e aguarda o giro cessar antes de avançar. Verifica novamente o erro
  lateral após ficar de frente para a parede. Se necessário, recua fora da
  abertura para corrigir a aproximação, no máximo duas vezes.
- O limite comandado passou de 0,08 m/s para **0,25 m/s na aproximação**, com
  limite de **0,18 m/s na entrada**, redução progressiva e avanço final de cerca
  de 0,035 m/s. Não é a velocidade nominal de missão `WP_SPEED`.
- Durante o docking, aplica `ATC_STR_RAT_FF=0.7`, `ATC_STR_RAT_P=0.7` e
  `ATC_STR_RAT_I=0`. Esses ganhos são temporários. Os ensaios confirmaram a
  restauração de FF=0.4, P=0.4, I=0.05, OA_TYPE=0 e AVOID_ENABLE=0, que eram os
  valores ativos nesta instância. A rotina salva os valores que encontrar antes
  da tentativa em `parametros_anteriores.txt`.
- A frenagem foi ajustada à resposta medida: margem de 0,025 m, latência de
  0,2 s e desaceleração de referência de 0,3 m/s². A região final aciona HOLD;
  o desarmamento exige parada confirmada e três imagens recentes cobrindo
  pelo menos 0,4 s. Tolerâncias visuais: 6 cm longitudinal, 8 cm lateral e 6°.
- O controle LiDAR anterior foi separado em `docking_lidar/alinhamento.py`.
  Os ganhos e a frenagem revisados são específicos da entrada por QR.

## Método de teste e limites

Os comandos foram enviados pelo MAVProxy local, porta 14551, ao ArduRover SITL.
O controlador recebeu imagem, CameraInfo e juntas do ROS 2 Jazzy; não recebeu
posição verdadeira do Gazebo. A câmera permaneceu em 640×360.

O avaliador confirmou SITL, HOLD e desarmamento antes de reposicionar o modelo.
Esperou a recuperação do estimador após esse reposicionamento e armou pelo
procedimento normal, sem desabilitar PreArm. Durante cada tentativa, acompanhou
`/world/gravel_plane/pose/info` e interromperia o teste se o envelope de
0,60 × 0,54 m do rover cruzasse os limites internos verificados da estação.
Nenhuma das seis execuções finais acionou essa interrupção.

Para aprovar, exigiu: término normal, HOLD e desarmamento confirmados, erro
físico longitudinal/lateral de até 8 cm, erro angular de até 6°, missão preservada
e leitura dos parâmetros restaurados. Os resultados ficaram abaixo desses
limites. HOME (item 0) é atualizado normalmente pelo ArduPilot ao armar; os
oito itens de missão seguintes foram comparados e permaneceram iguais.

Os ensaios anteriores à revisão final **não foram todos bem-sucedidos**:
foram identificados erro transitório do EKF após teletransporte, erro angular na
pose do QR, oscilação devido à pouca resposta de giro durante avanço e parada
6,6 cm antes do centro. Esses ensaios foram interrompidos ou reprovados, e os
registros permanecem em `python/run/docking_validation/`. A série aprovada é
exclusivamente `20261109_181935`, com os arquivos do manifesto acima.

A validação cobre a estação `container_gravel_plane`, o perfil ativo sem desvio
e os seis estados iniciais da tabela, com área de aproximação e recuo livre.
Não valida o intervalo inteiro de ativação de 8 m, outros obstáculos no trajeto,
contato elétrico de carga, saída de ré, entrada por LiDAR ou hardware real.

## Executar

Com Gazebo, SITL e MAVProxy ativos, reinicie o monitor de entrada:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh docking-qr
```

Ele observa a telemetria e inicia quando o rover estiver parado, armado, em
GUIDED e a menos de 8 m da estação cadastrada. Não cria nem envia missão.
O limite de 8 m permite iniciar a tentativa; ainda é necessário reconhecer o QR.
Ao terminar, confirma HOLD e desarma. Para reduzir a velocidade:

```bash
./ros2/service.sh docking-qr --velocidade 0.15
```

Para reproduzir a bateria, encerre outros controladores de docking e execute
**somente neste SITL**. O comando abaixo reposiciona e movimenta o rover simulado:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
python python/tests/validate_docking_series.py --execute
```

O comportamento e os comandos dos demais serviços estão no
[manual de docking](<README DOCKING.md>).
